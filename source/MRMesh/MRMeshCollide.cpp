#include "MRMeshCollide.h"
#include "MRAABBTree.h"
#include "MRMesh.h"
#include "MRTriangleIntersection.h"
#include "MREnums.h"
#include "MRTimer.h"
#include "MRGTest.h"
#include "MRPch/MRTBB.h"
#include "MRExpected.h"
#include "MRProcessSelfTreeSubtasks.h"
#include <atomic>
#include <thread>

namespace MR
{

std::vector<FaceFace> findCollidingTriangles( const MeshPart & a, const MeshPart & b, const AffineXf3f * rigidB2A, bool firstIntersectionOnly )
{
    MR_TIMER;

    std::vector<FaceFace> res;
    const AABBTree & aTree = a.mesh.getAABBTree();
    const AABBTree & bTree = b.mesh.getAABBTree();
    if ( aTree.nodes().empty() || bTree.nodes().empty() )
        return res;

    NodeBitSet aNodes, bNodes;
    NodeBitSet* aNodesPtr{nullptr}, * bNodesPtr{nullptr};
    if ( a.region )
    {
        aNodes = aTree.getNodesFromLeaves( *a.region );
        aNodesPtr = &aNodes;
    }
    if ( b.region )
    {
        bNodes = bTree.getNodesFromLeaves( *b.region );
        bNodesPtr = &bNodes;
    }

    std::vector<NodeNode> subtasks{ { NodeId{ 0 }, NodeId{ 0 } } };

    while( !subtasks.empty() )
    {
        const auto s = subtasks.back();
        subtasks.pop_back();

        if ( aNodesPtr && !aNodes.test( s.aNode ) )
            continue;
        if ( bNodesPtr && !bNodes.test( s.bNode ) )
            continue;

        const auto & aNode = aTree[s.aNode];
        const auto & bNode = bTree[s.bNode];

        const auto overlap = aNode.box.intersection( transformed( bNode.box, rigidB2A ) );
        if ( !overlap.valid() )
            continue;

        if ( aNode.leaf() && bNode.leaf() )
        {
            const auto aFace = aNode.leafId();
            const auto bFace = bNode.leafId();
            res.emplace_back( aFace, bFace );
            continue;
        }

        if ( !aNode.leaf() && ( bNode.leaf() || aNode.box.volume() >= bNode.box.volume() ) )
        {
            // split aNode
            subtasks.push_back( { aNode.l, s.bNode } );
            subtasks.push_back( { aNode.r, s.bNode } );
        }
        else
        {
            assert( !bNode.leaf() );
            // split bNode
            subtasks.push_back( { s.aNode, bNode.l } );
            subtasks.push_back( { s.aNode, bNode.r } );
        }
    }

    std::atomic<int> firstIntersection{ (int)res.size() };
    tbb::parallel_for( tbb::blocked_range<int>( 0, (int)res.size() ),
        [&]( const tbb::blocked_range<int>& range )
    {
        for ( int i = range.begin(); i < range.end(); ++i )
        {
            int knownIntersection = firstIntersection.load( std::memory_order_relaxed );
            if ( firstIntersectionOnly && knownIntersection < i )
                break;
            Vector3f av[3], bv[3];
            a.mesh.getTriPoints( res[i].aFace, av[0], av[1], av[2] );
            b.mesh.getTriPoints( res[i].bFace, bv[0], bv[1], bv[2] );
            if ( rigidB2A )
            {
                bv[0] = (*rigidB2A)( bv[0] );
                bv[1] = (*rigidB2A)( bv[1] );
                bv[2] = (*rigidB2A)( bv[2] );
            }
            if ( doTrianglesIntersect( Vector3d{ av[0] }, Vector3d{ av[1] }, Vector3d{ av[2] }, Vector3d{ bv[0] }, Vector3d{ bv[1] }, Vector3d{ bv[2] } ) )
            {
                if ( firstIntersectionOnly )
                {
                    while ( knownIntersection > i && !firstIntersection.compare_exchange_strong( knownIntersection, i ) ) { }
                    break;
                }
            }
            else
            {
                res[i].aFace = FaceId{}; //invalidate
            }
        }
    } );

    if ( firstIntersectionOnly )
    {
        int knownIntersection = firstIntersection.load( std::memory_order_relaxed );
        if ( knownIntersection < res.size() )
        {
            res[0] = res[knownIntersection];
            res.erase( res.begin() + 1, res.end() );
        }
        else
            res.clear();
    }
    else
    {
        res.erase( std::remove_if( res.begin(), res.end(), []( const FaceFace & ff ) { return !ff.aFace.valid(); } ), res.end() );
    }

    return res;
}

std::pair<FaceBitSet, FaceBitSet> findCollidingTriangleBitsets( const MeshPart& a, const MeshPart& b,
    const AffineXf3f* rigidB2A )
{
    const auto pairs = findCollidingTriangles( a, b, rigidB2A );
    FaceId aMax, bMax;
    for ( const auto & p : pairs )
    {
        aMax = std::max( aMax, p.aFace );
        bMax = std::max( bMax, p.bFace );
    }

    std::pair<FaceBitSet, FaceBitSet> res;
    res.first.resize( aMax + 1 );
    res.second.resize( bMax + 1 );
    for ( const auto & p : pairs )
    {
        res.first.set( p.aFace );
        res.second.set( p.bFace );
    }
    return res;
}

inline std::pair<int, int> sharedVertex( const VertId av[3], const VertId bv[3] )
{
    for ( int i = 0; i < 3; ++i )
    {
        for ( int j = 0; j < 3; ++j )
        {
            if ( av[i] == bv[j] )
                return { i, j };
        }
    }
    return { -1, -1 };
}

Expected<bool> findSelfCollidingTriangles(
    const MeshPart& mp,
    std::vector<FaceFace> * outCollidingPairs,
    ProgressCallback cb,
    const Face2RegionMap * regionMap,
    bool touchIsIntersection )
{
    MR_TIMER;
    const AABBTree & tree = mp.mesh.getAABBTree();
    if ( tree.nodes().empty() )
        return false;

    auto sb = subprogress( cb, 0, 0.08f );

    // sequentially subdivide full task on smaller subtasks;
    // they shall be not too many for this subdivision not to take too long;
    // and they shall be not too few for enough parallelism later
    std::vector<NodeNode> subtasks{ { NodeId{ 0 }, NodeId{ 0 } } }, nextSubtasks, leafTasks;
    for( int i = 0; i < 16 && !subtasks.empty(); ++i ) // 16 -> will produce at most 2^16 subtasks
    {
        processSelfSubtasks( tree, subtasks, nextSubtasks,
            [&leafTasks]( const NodeNode & s ) { leafTasks.push_back( s ); return Processing::Continue; },
            [](const Box3f& lBox, const Box3f& rBox ){ return lBox.intersects( rBox ) ? Processing::Continue : Processing::Stop; });
        subtasks.swap( nextSubtasks );

        if ( !reportProgress( sb, i / 16.0f ) )
            return unexpectedOperationCanceled();
    }
    subtasks.insert( subtasks.end(), leafTasks.begin(), leafTasks.end() );

    sb = subprogress( cb, 0.08f, 0.92f );

    std::vector<std::vector<FaceFace>> subtaskRes( subtasks.size() );

    auto mainThreadId = std::this_thread::get_id();
    std::atomic<bool> keepGoing{ true };
    std::atomic<size_t> numDone;
    // checks subtasks in parallel
    tbb::parallel_for( tbb::blocked_range<size_t>( 0, subtasks.size() ),
        [&]( const tbb::blocked_range<size_t>& range )
    {
        std::vector<NodeNode> mySubtasks;
        for ( auto is = range.begin(); is < range.end(); ++is )
        {
            if ( sb && !keepGoing.load( std::memory_order_relaxed ) )
                break;

            mySubtasks.push_back( subtasks[is] );
            std::vector<FaceFace> myRes;
            processSelfSubtasks( tree, mySubtasks, mySubtasks,
                [&tree, &mp, &myRes, regionMap, outCollidingPairs, &keepGoing, touchIsIntersection]( const NodeNode & s )
                {
                    const auto & aNode = tree[s.aNode];
                    const auto & bNode = tree[s.bNode];
                    const auto aFace = aNode.leafId();
                    if ( mp.region && !mp.region->test( aFace ) )
                        return Processing::Continue;
                    const auto bFace = bNode.leafId();
                    if ( mp.region && !mp.region->test( bFace ) )
                        return Processing::Continue;
                    if ( regionMap && ( *regionMap )[aFace] != ( *regionMap )[bFace] )
                        return Processing::Continue;

                    VertId av[3], bv[3];
                    Vector3d ap[3], bp[3];

                    auto se = mp.mesh.topology.sharedEdge( aFace, bFace );
                    if ( se )
                    {
                        mp.mesh.topology.getLeftTriVerts( se, av[0], av[1], av[2] );
                        mp.mesh.topology.getLeftTriVerts( se.sym(), bv[0], bv[1], bv[2] );
                    }
                    else
                    {
                        mp.mesh.topology.getTriVerts( aFace, av[0], av[1], av[2] );
                        mp.mesh.topology.getTriVerts( bFace, bv[0], bv[1], bv[2] );
                    }
                    for ( int j = 0; j < 3; ++j )
                    {
                        ap[j] = Vector3d{ mp.mesh.points[av[j]] };
                        bp[j] = Vector3d{ mp.mesh.points[bv[j]] };
                    }
                    if ( se )
                    {
                        // check coplanar
                        if ( !touchIsIntersection || ( !isPointInTriangle( bp[2], ap[0], ap[1], ap[2] ) && !isPointInTriangle( ap[2], bp[0], bp[1], bp[2] ) ) )
                            return Processing::Continue;
                        // else not coplanar
                    }
                    else if ( auto sv = sharedVertex( av, bv ); sv.first >= 0 )
                    {
                        // shared vertex
                        const int j = sv.first;
                        const int k = sv.second;
                        if ( !doTriangleSegmentIntersect( ap[0], ap[1], ap[2], bp[( k + 1 ) % 3], bp[( k + 2 ) % 3] ) &&
                             !doTriangleSegmentIntersect( bp[0], bp[1], bp[2], ap[( j + 1 ) % 3], ap[( j + 2 ) % 3] ) )
                        {
                            // check touching too
                            if ( !touchIsIntersection ||
                                  ( !isPointInTriangle( ap[( j + 1 ) % 3], bp[0], bp[1], bp[2] ) &&
                                    !isPointInTriangle( ap[( j + 2 ) % 3], bp[0], bp[1], bp[2] ) &&
                                    !isPointInTriangle( bp[( k + 1 ) % 3], ap[0], ap[1], ap[2] ) &&
                                    !isPointInTriangle( bp[( k + 2 ) % 3], ap[0], ap[1], ap[2] ) ) )
                                return Processing::Continue;
                            // else not touching
                        }
                    }
                    else if ( !doTrianglesIntersectExt( ap[0], ap[1], ap[2], bp[0], bp[1], bp[2] ) )
                    {
                        if ( !touchIsIntersection )
                            return Processing::Continue;
                        // check touching too
                        bool touching = false;
                        for ( int i = 0; i < 3; ++i )
                        {
                            if ( isPointInTriangle( ap[i], bp[0], bp[1], bp[2] ) || isPointInTriangle( bp[i], ap[0], ap[1], ap[2] ) )
                            {
                                touching = true;
                                break;
                            }
                        }
                        if ( !touching )
                            return Processing::Continue;
                    }
                    myRes.emplace_back( aFace, bFace );
                    if ( !outCollidingPairs )
                    {
                        keepGoing.store( false, std::memory_order_relaxed );
                        return Processing::Stop;
                    }
                    return Processing::Continue;
                },
                [](const Box3f& lBox, const Box3f& rBox ){ return lBox.intersects( rBox ) ? Processing::Continue : Processing::Stop; }
            );

            subtaskRes[is] = std::move( myRes );
        }

        if ( cb )
            numDone += range.size();

        if ( sb && std::this_thread::get_id() == mainThreadId )
        {
            if ( !reportProgress( sb, float( numDone ) / subtasks.size() ) )
                keepGoing.store( false, std::memory_order_relaxed );
        }
    } );

    // unite results from sub-trees into final vector
    size_t cols = 0;
    for ( const auto & s : subtaskRes )
        cols += s.size();

    if ( !outCollidingPairs && cols > 0 )
        return true; // even if keepGoing = false

    if ( !keepGoing.load( std::memory_order_relaxed ) || !reportProgress( sb, 1.0f ) )
        return unexpectedOperationCanceled();

    if ( outCollidingPairs )
    {
        outCollidingPairs->reserve( outCollidingPairs->size() + cols );
        for ( const auto & s : subtaskRes )
            outCollidingPairs->insert( outCollidingPairs->end(), s.begin(), s.end() );
    }

    if ( !reportProgress( cb, 1.0f ) )
        return unexpectedOperationCanceled();

    return cols > 0;
}

Expected<std::vector<FaceFace>> findSelfCollidingTriangles( const MeshPart& mp, ProgressCallback cb,
    const Face2RegionMap* regionMap,
    bool touchIsIntersection )
{
    std::vector<FaceFace> res;
    auto exp = findSelfCollidingTriangles( mp, &res, cb, regionMap, touchIsIntersection );
    if ( !exp )
        return unexpected( std::move( exp.error() ) );
    return res;
}

Expected<FaceBitSet> findSelfCollidingTrianglesBS( const MeshPart& mp, ProgressCallback cb, const Face2RegionMap* regionMap, bool touchIsIntersection )
{
    MR_TIMER;

    auto ffs = findSelfCollidingTriangles( mp, cb, regionMap, touchIsIntersection );
    if ( !ffs.has_value() )
        return unexpected( ffs.error() );

    FaceBitSet res;
    for ( const auto & ff : ffs.value() )
    {
        res.autoResizeSet( ff.aFace );
        res.autoResizeSet( ff.bFace );
    }

    return res;
}

bool isInside( const MeshPart & a, const MeshPart & b, const AffineXf3f * rigidB2A )
{
    auto cols = findCollidingTriangles( a, b, rigidB2A, true );
    if ( !cols.empty() )
        return false; // meshes intersect

    return isNonIntersectingInside( a, b, rigidB2A );
}

bool isNonIntersectingInside( const MeshPart& a, const MeshPart& b, const AffineXf3f* rigidB2A )
{
    auto aFace = a.mesh.topology.getFaceIds( a.region ).find_first();
    return isNonIntersectingInside( a.mesh, aFace, b, rigidB2A );
}

bool isNonIntersectingInside( const Mesh& a, FaceId aFace, const MeshPart& b, const AffineXf3f* rigidB2A /*= nullptr */ )
{
    if ( !aFace )
        return true; //consider empty mesh always inside

    Vector3f aPoint = a.triCenter( aFace );
    if ( rigidB2A )
        aPoint = rigidB2A->inverse()( aPoint );

    auto signDist = b.mesh.signedDistance( aPoint, FLT_MAX, b.region );
    return signDist && signDist < 0;
}

TEST( MRMesh, DegenerateTrianglesIntersect )
{
    Vector3f a{-24.5683002f,-17.7052994f,-21.3701000f};
    Vector3f b{-24.6611996f,-17.7504997f,-21.3423004f};
    Vector3f c{-24.6392994f,-17.7071991f,-21.3542995f};

    Vector3f d{-24.5401993f,-17.7504997f,-21.3390007f};
    Vector3f e{-24.5401993f,-17.7504997f,-21.3390007f};
    Vector3f f{-24.5862007f,-17.7504997f,-21.3586998f};

    bool intersection = doTrianglesIntersect(
        Vector3d{a}, Vector3d{b}, Vector3d{c},
        Vector3d{d}, Vector3d{e}, Vector3d{f} );

    // in float arithmetic this test fails unfortunately

    EXPECT_FALSE( intersection );
}

} //namespace MR
