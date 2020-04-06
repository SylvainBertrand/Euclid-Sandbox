package us.ihmc.euclid.visualizers;

import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryFactories;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class STPBox3D implements SupportingVertexHolder
{
   private final double largeRadius = 2.0;
   private final double smallRadius = 0.05;

   private final Box3DReadOnly box3D;
   private final Vector3D supportDirectionLocal = new Vector3D();
   private final STPBox3DFace face = new STPBox3DFace();
   private final Point3D supportingVertex = new Point3D();

   public STPBox3D(Box3DReadOnly box3D)
   {
      this.box3D = box3D;
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (!box3D.getPose().getRotation().isIdentity())
      {
         box3D.getPose().inverseTransform(supportDirection, supportDirectionLocal);
         supportDirection = supportDirectionLocal;
      }

      double faceXPlusDot = supportDirection.dot(Axis3D.X);
      double faceYPlusDot = supportDirection.dot(Axis3D.Y);
      double faceZPlusDot = supportDirection.dot(Axis3D.Z);

      double faceXAbsDot = Math.abs(faceXPlusDot);
      double faceYAbsDot = Math.abs(faceYPlusDot);
      double faceZAbsDot = Math.abs(faceZPlusDot);

      if (faceXAbsDot > faceYAbsDot)
      {
         if (faceXAbsDot > faceZAbsDot)
         { // Closest is one of the 2 x-faces
            setupXFace(faceXPlusDot > 0.0, face);
         }
         else
         { // Closest is one of the 2 z-faces
            setupZFace(faceZPlusDot > 0.0, face);
         }
      }
      else if (faceYAbsDot > faceZAbsDot)
      { // Closest is one of the 2 y-faces
         setupYFace(faceYPlusDot > 0.0, face);
      }
      else
      { // Closest is one of the 2 z-faces
         setupZFace(faceZPlusDot > 0.0, face);
      }

      if (!getFaceSupportingVertex(supportDirection, face, supportingVertexToPack))
      {
         if (!getBestEdgeSupportingVertex(supportDirection, face, supportingVertexToPack))
         {
            EuclidShapeTools.supportingVertexBox3D(supportDirection, box3D.getSize(), supportingVertex);
            EuclidShapeTools.supportingVertexSphere3D(supportDirection, supportingVertex, smallRadius, supportingVertexToPack);
         }
      }

      if (!box3D.getPose().getRotation().isIdentity())
      {
         box3D.getPose().transform(supportingVertexToPack);
      }

      return true;
   }

   private final Point3D faceSphereCenter = new Point3D();

   private boolean getFaceSupportingVertex(Vector3DReadOnly supportDirection, STPBox3DFace face, Point3DBasics supportingVertexToPack)
   {
      EuclidGeometryTools.sphere3DPositionFromThreePoints(face.v0, face.v1, face.v2, largeRadius - smallRadius, faceSphereCenter);
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);
      return face.isPointDirectlyAboveOrBelow(supportingVertexToPack);
   }

   private boolean getBestEdgeSupportingVertex(Vector3DReadOnly supportDirection, STPBox3DFace bestFace, Point3DBasics supportingVertexToPack)
   {
      LineSegment3DReadOnly bestEdge = null;
      double bestEdgeDot = Double.NEGATIVE_INFINITY;

      for (LineSegment3DReadOnly edge : face.edges)
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

   private void setupXFace(boolean xPlus, STPBox3DFace face)
   {
      double halfSizeX = 0.5 * box3D.getSizeX();
      double halfSizeY = 0.5 * box3D.getSizeY();
      double halfSizeZ = 0.5 * box3D.getSizeZ();

      if (xPlus)
      {
         face.faceNormal.set(Axis3D.X);
         face.v0.set(halfSizeX, halfSizeY, halfSizeZ);
         face.v1.set(halfSizeX, -halfSizeY, -halfSizeZ);
         face.v2.set(halfSizeX, halfSizeY, -halfSizeZ);
         face.v3.set(halfSizeX, -halfSizeY, halfSizeZ);
      }
      else
      {
         face.faceNormal.setAndNegate(Axis3D.X);
         face.v0.set(-halfSizeX, halfSizeY, halfSizeZ);
         face.v1.set(-halfSizeX, -halfSizeY, -halfSizeZ);
         face.v2.set(-halfSizeX, -halfSizeY, halfSizeZ);
         face.v3.set(-halfSizeX, halfSizeY, -halfSizeZ);
      }
   }

   private void setupYFace(boolean yPlus, STPBox3DFace face)
   {
      double halfSizeX = 0.5 * box3D.getSizeX();
      double halfSizeY = 0.5 * box3D.getSizeY();
      double halfSizeZ = 0.5 * box3D.getSizeZ();

      if (yPlus)
      {
         face.faceNormal.set(Axis3D.Y);
         face.v0.set(halfSizeX, halfSizeY, halfSizeZ);
         face.v1.set(-halfSizeX, halfSizeY, -halfSizeZ);
         face.v2.set(-halfSizeX, halfSizeY, halfSizeZ);
         face.v3.set(halfSizeX, halfSizeY, -halfSizeZ);
      }
      else
      {
         face.faceNormal.setAndNegate(Axis3D.Y);
         face.v0.set(halfSizeX, -halfSizeY, halfSizeZ);
         face.v1.set(-halfSizeX, -halfSizeY, -halfSizeZ);
         face.v2.set(halfSizeX, -halfSizeY, -halfSizeZ);
         face.v3.set(-halfSizeX, -halfSizeY, halfSizeZ);
      }
   }

   private void setupZFace(boolean zPlus, STPBox3DFace face)
   {
      double halfSizeX = 0.5 * box3D.getSizeX();
      double halfSizeY = 0.5 * box3D.getSizeY();
      double halfSizeZ = 0.5 * box3D.getSizeZ();

      if (zPlus)
      {
         face.faceNormal.set(Axis3D.Z);
         face.v0.set(halfSizeX, halfSizeY, halfSizeZ);
         face.v1.set(-halfSizeX, -halfSizeY, halfSizeZ);
         face.v2.set(halfSizeX, -halfSizeY, halfSizeZ);
         face.v3.set(-halfSizeX, halfSizeY, halfSizeZ);
      }
      else
      {
         face.faceNormal.setAndNegate(Axis3D.Z);
         face.v0.set(halfSizeX, halfSizeY, -halfSizeZ);
         face.v1.set(-halfSizeX, -halfSizeY, -halfSizeZ);
         face.v2.set(-halfSizeX, halfSizeY, -halfSizeZ);
         face.v3.set(halfSizeX, -halfSizeY, -halfSizeZ);
      }
   }

   private static class STPBox3DFace
   {
      private final Vector3D faceNormal = new Vector3D();
      private final Point3D v0 = new Point3D();
      private final Point3D v1 = new Point3D();
      private final Point3D v2 = new Point3D();
      private final Point3D v3 = new Point3D();
      private final LineSegment3DReadOnly e0 = EuclidGeometryFactories.newLinkedLineSegment3DReadOnly(v0, v1);
      private final LineSegment3DReadOnly e1 = EuclidGeometryFactories.newLinkedLineSegment3DReadOnly(v1, v2);
      private final LineSegment3DReadOnly e2 = EuclidGeometryFactories.newLinkedLineSegment3DReadOnly(v2, v3);
      private final LineSegment3DReadOnly e3 = EuclidGeometryFactories.newLinkedLineSegment3DReadOnly(v3, v0);
      private final List<LineSegment3DReadOnly> edges = Arrays.asList(e0, e1, e2, e3);

      /**
       * Tests whether the query is located directly above or below this face, such its projection would
       * be located inside this face.
       *
       * @param query the coordinates of the query. Not modified.
       * @return {@code true} if the query is located either directly above or below this face,
       *         {@code false} otherwise.
       */
      public boolean isPointDirectlyAboveOrBelow(Point3DReadOnly query)
      {
         if (canObserverSeeEdge(query, e0))
            return false;
         if (canObserverSeeEdge(query, e1))
            return false;
         if (canObserverSeeEdge(query, e2))
            return false;
         if (canObserverSeeEdge(query, e3))
            return false;
         return true;
      }

      public boolean canObserverSeeEdge(Point3DReadOnly observer, LineSegment3DReadOnly edge)
      {
         return EuclidPolytopeTools.isPoint3DOnLeftSideOfLine3D(observer, edge.getFirstEndpoint(), edge.getSecondEndpoint(), faceNormal);
      }
   }
}
