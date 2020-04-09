package us.ihmc.euclid.visualizers;

import java.util.List;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.visualizers.STPFace3D.STPHalfEdge3D;

public class STPConvexPolytope3D implements SupportingVertexHolder
{
   private final double largeRadius = 15.00;
   private final double smallRadius = 0.005;

   private final ConvexPolytope3DReadOnly polytope;

   public STPConvexPolytope3D(ConvexPolytope3DReadOnly polytope)
   {
      this.polytope = polytope;
   }

   public ConvexPolytope3DReadOnly getPolytope()
   {
      return polytope;
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      Vertex3DReadOnly bestVertex = polytope.getSupportingVertex(supportDirection);

      if (bestVertex == null)
         return false;

      List<STPFace3D> bestSTPFaces = findBestFace(supportDirection, bestVertex);

      if (getFaceSupportingVertex(supportDirection, bestSTPFaces, bestVertex, supportingVertexToPack))
         return true;

      if (getBestEdgeSupportingVertex(supportDirection, bestSTPFaces, bestVertex, supportingVertexToPack))
         return true;

      EuclidShapeTools.supportingVertexSphere3D(supportDirection, bestVertex, smallRadius, supportingVertexToPack);
      return true;
   }

   private List<STPFace3D> findBestFace(Vector3DReadOnly supportDirection, Vertex3DReadOnly bestVertex)
   {
      Face3DReadOnly bestFace = bestVertex.getAssociatedEdge(0).getFace();
      double bestFaceDot = bestFace.getNormal().dot(supportDirection);

      for (int i = 1; i < bestVertex.getNumberOfAssociatedEdges(); i++)
      {
         HalfEdge3DReadOnly candidateEdge = bestVertex.getAssociatedEdge(i);
         Face3DReadOnly candidateFace = candidateEdge.getFace();
         double candidateDot = candidateFace.getNormal().dot(supportDirection);

         if (candidateDot > bestFaceDot)
         {
            bestFaceDot = candidateDot;
            bestFace = candidateFace;
         }
      }

      return STPFace3D.newSTPFace3Ds(bestFace);
   }

   private final Point3D faceSphereCenter = new Point3D();

   private boolean getFaceSupportingVertex(Vector3DReadOnly supportDirection, List<STPFace3D> bestFaces, Vertex3DReadOnly bestVertex,
                                           Point3DBasics supportingVertexToPack)
   {
      for (STPFace3D stpFace : bestFaces)
      {
         if (!stpFace.contains(bestVertex))
            continue;

         if (getTriangleSupportingVertex(supportDirection, stpFace, supportingVertexToPack))
            return true;
      }
      return false;
   }

   private boolean getTriangleSupportingVertex(Vector3DReadOnly supportDirection, STPFace3D face, Point3DBasics supportingVertexToPack)
   {
      EuclidGeometryTools.sphere3DPositionFromThreePoints(face.getV0(), face.getV1(), face.getV2(), largeRadius - smallRadius, faceSphereCenter);
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);
      return face.isPointDirectlyAboveOrBelow(supportingVertexToPack);
   }

   private boolean getBestEdgeSupportingVertex(Vector3DReadOnly supportDirection, List<STPFace3D> bestFaces, Vertex3DReadOnly bestVertex,
                                               Point3DBasics supportingVertexToPack)
   {
      STPHalfEdge3D bestEdge = null;
      double bestEdgeDot = Double.NEGATIVE_INFINITY;

      for (STPFace3D face : bestFaces)
      {
         if (!face.contains(bestVertex))
            continue;

         for (STPHalfEdge3D edge : face.getEdges())
         {
            if (!edge.contains(bestVertex))
               continue;

            double candidateDot = TupleTools.dot(edge.midpoint(), supportDirection);

            if (candidateDot > bestEdgeDot)
            {
               bestEdgeDot = candidateDot;
               bestEdge = edge;
            }
         }
      }

      return getEdgeSupportingVertex(supportDirection, bestEdge, supportingVertexToPack);
   }

   private final Point3D edgeTorusCenter = new Point3D();
   private final Vector3D edgeTorusAxis = new Vector3D();

   private boolean getEdgeSupportingVertex(Vector3DReadOnly supportDirection, STPHalfEdge3D edge, Point3DBasics supportingVertexToPack)
   {
      edgeTorusCenter.add(edge.getOrigin(), edge.getDestination());
      edgeTorusCenter.scale(0.5);
      edgeTorusAxis.sub(edge.getDestination(), edge.getOrigin());

      double radius = largeRadius - smallRadius;
      double torusRadius = Math.sqrt(radius * radius - 0.25 * edgeTorusAxis.lengthSquared());
      double torusTubeRadius = largeRadius;

      EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection, edgeTorusCenter, edgeTorusAxis, torusRadius, torusTubeRadius, supportingVertexToPack);

      if (!edge.isBetweenEndpoints(supportingVertexToPack))
         return false;
      if (edge.getFace().isPointDirectlyAboveOrBelow(supportingVertexToPack))
         return false;
      if (edge.getTwin() != null && edge.getTwin().getFace().isPointDirectlyAboveOrBelow(supportingVertexToPack))
         return false;
      return true;
   }
}
