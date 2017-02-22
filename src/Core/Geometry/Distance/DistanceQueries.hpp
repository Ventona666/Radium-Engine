#ifndef RADIUMENGINE_DISTANCE_QUERIES_HPP_
#define RADIUMENGINE_DISTANCE_QUERIES_HPP_

#include <Core/RaCore.hpp>
#include <Core/Math/LinearAlgebra.hpp>
#include <Core/Mesh/TriangleMesh.hpp>

/// Functions in this file are utilities to compute the distance between various geometric sets.
/// They always return the squared distance.


namespace Ra
{
    namespace Core
    {
        namespace DistanceQueries
        {
            //
            // Point-to-line distance
            //


            /// Return the squared distance from point Q to the line defined by point A and direction dir.
            inline RA_CORE_API Scalar pointToLineSq(const Vector3& q, const Vector3& a, const Vector3& dir);

            /// Project point Q on segment AB defined by point A and vector AB = (B -A).
            /// Return the parameter t in [0,1] which identifies the projected point.
            inline RA_CORE_API Scalar projectOnSegment(const Vector3& q, const Vector3& a, const Vector3& ab);

            /// Return the squared distance from point Q to the segment AB defined by point A and
            /// vector AB = (B - A).
            inline RA_CORE_API Scalar pointToSegmentSq(const Vector3& q, const Vector3& a, const Vector3& ab);

            //
            // Point-to-triangle distance
            //

            struct PointToTriangleOutput
            {
                enum Flags
                {
                    HIT_FACE = 0,
                    HIT_VERTEX = 1,
                    HIT_EDGE = 2,
                };

                PointToTriangleOutput() : meshPoint({0,0,0}), distanceSquared(std::numeric_limits<Scalar>::max()), flags(0){}

                Vector3 meshPoint; // the point hit on the mesh
                Scalar distanceSquared; // distance squared to the point
                uchar flags;     // Bits 0-1 : if the hit is a face (00) a vertex (01) or an edge (10)
                                 // bits 2-3 : if vertex, index of the hit vertex
                                 //            if edge, index of the edge's first vertex

                // Return if the hit is a face, a vertex or an edge
                inline Flags getHitPrimitive() const { return Flags(flags & 0x3u); }
                // Return the index of the hit vertex or the first vertex of the hit edge.
                // If hit is a face hit, returns 0
                uint getHitIndex() const
                {
                    return (flags & 0xcu)>>2;
                }
            };



            /// Returns the squared distance d from a query point Q to the triangle ABC.
            /// triPoint T is computed as the closest point on the triangle.
            /// Thus ||QT|| = d.
            /// Triangle ABC must not be degenerate.
            inline RA_CORE_API PointToTriangleOutput
            pointToTriSq(const Vector3& q, const Vector3& a, const Vector3& b, const Vector3& c);

        } // ns Distance queries
    }// ns Core
} // ns Ra

#include <Core/Geometry/Distance/DistanceQueries.inl>

#endif //RADIUMENGINE_DISTANCE_QUERIES_HPP_
