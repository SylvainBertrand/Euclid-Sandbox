package us.ihmc.euclid.visualizers;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class STPBox3D implements SupportingVertexHolder
{
   private final double largeRadius = 10.0;
   private final double smallRadius = 0.01;

   private final Box3DReadOnly box3D;

   public STPBox3D(Box3DReadOnly box3D)
   {
      this.box3D = box3D;
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      BoxPolytope3DView boxPolytope = box3D.asConvexPolytope();

      Vertex3DReadOnly bestVertex = boxPolytope.getSupportingVertex(supportDirection);
      Face3DReadOnly bestFace = findBestFace(supportDirection, bestVertex);

      if (getFaceSupportingVertex(supportDirection, bestFace, supportingVertexToPack))
         return true;
      if (getBestEdgeSupportingVertex(supportDirection, bestFace, supportingVertexToPack))
         return true;
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, bestVertex, smallRadius, supportingVertexToPack);
      return true;
   }

   private Face3DReadOnly findBestFace(Vector3DReadOnly supportDirection, Vertex3DReadOnly bestVertex)
   {
      Face3DReadOnly bestFace = bestVertex.getAssociatedEdge(0).getFace();
      double bestFaceDot = bestFace.getNormal().dot(supportDirection);

      for (int i = 1; i < bestVertex.getNumberOfAssociatedEdges(); i++)
      {
         Face3DReadOnly candidateFace = bestVertex.getAssociatedEdge(i).getFace();
         double candidateDot = candidateFace.getNormal().dot(supportDirection);

         if (candidateDot > bestFaceDot)
         {
            bestFaceDot = candidateDot;
            bestFace = candidateFace;
         }
      }

      return bestFace;
   }

   private final Point3D faceSphereCenter = new Point3D();

   private boolean getFaceSupportingVertex(Vector3DReadOnly supportDirection, Face3DReadOnly face, Point3DBasics supportingVertexToPack)
   {
      EuclidGeometryTools.sphere3DPositionFromThreePoints(face.getVertex(0), face.getVertex(1), face.getVertex(2), largeRadius - smallRadius, faceSphereCenter);
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);
      return face.isPointDirectlyAboveOrBelow(supportingVertexToPack);
   }

   private boolean getBestEdgeSupportingVertex(Vector3DReadOnly supportDirection, Face3DReadOnly bestFace, Point3DBasics supportingVertexToPack)
   {
      LineSegment3DReadOnly bestEdge = null;
      double bestEdgeDot = Double.NEGATIVE_INFINITY;

      for (HalfEdge3DReadOnly edge : bestFace.getEdges())
      {
         double candidateDot = TupleTools.dot(edge.midpoint(), supportDirection);

         if (candidateDot > bestEdgeDot)
         {
            bestEdgeDot = candidateDot;
            bestEdge = edge;
         }
      }

      return getEdgeSupportingVertex(supportDirection, bestEdge, supportingVertexToPack);
   }

   private final Point3D edgeTorusCenter = new Point3D();
   private final Vector3D edgeTorusAxis = new Vector3D();

   private boolean getEdgeSupportingVertex(Vector3DReadOnly supportDirection, LineSegment3DReadOnly edge, Point3DBasics supportingVertexToPack)
   {
      edgeTorusCenter.add(edge.getFirstEndpoint(), edge.getSecondEndpoint());
      edgeTorusCenter.scale(0.5);
      edgeTorusAxis.sub(edge.getSecondEndpoint(), edge.getFirstEndpoint());

      double radius = largeRadius - smallRadius;
      double torusRadius = Math.sqrt(radius * radius - 0.25 * edgeTorusAxis.lengthSquared());
      double torusTubeRadius = largeRadius;

      EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection, edgeTorusCenter, edgeTorusAxis, torusRadius, torusTubeRadius, supportingVertexToPack);

      return edge.isBetweenEndpoints(supportingVertexToPack);
   }
}
