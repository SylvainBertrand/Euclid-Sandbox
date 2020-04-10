package us.ihmc.euclid.visualizers;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class STPRamp3D implements SupportingVertexHolder
{
   private final double largeRadius = 2.0;
   private final double smallRadius = 0.05;

   private final Ramp3DReadOnly ramp3D;
   private final Vector3D supportDirectionLocal = new Vector3D();
   private final Point3D faceSphereCenter = new Point3D();
   private final Point3D closestEdgeCenter = new Point3D();
   private final Vector3D closestEdgeAxis = new Vector3D();
   private final Point3D closestVertexCenter = new Point3D();

   private final Point3D edgeCandidate1 = new Point3D();
   private final Point3D edgeCandidate2 = new Point3D();
   private final Point3D edgeCandidate3 = new Point3D();

   public STPRamp3D(Ramp3DReadOnly ramp3D)
   {
      this.ramp3D = ramp3D;
   }

   private enum Face
   {
      RAMP, X_MAX, Y_MIN, Y_MAX, Z_MIN
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (ramp3D.getPose().hasRotation())
      {
         ramp3D.getPose().inverseTransform(supportDirection, supportDirectionLocal);
         supportDirection = supportDirectionLocal;
      }

      Vector3DReadOnly rampSurfaceNormal = ramp3D.getRampSurfaceNormal();
      Vector3DReadOnly size = ramp3D.getSize();

      double rampDot = supportDirection.dot(rampSurfaceNormal);
      double faceXPlusDot = supportDirection.getX();
      double faceYPlusDot = supportDirection.getY();
      double faceZPlusDot = supportDirection.getZ();
      double faceYAbsDot = Math.abs(faceYPlusDot);
      boolean isFaceYMaxCloser = faceYPlusDot > 0.0;

      Face firstClosestFace;

      faceSphereCenter.setToZero();
      closestEdgeAxis.setToZero();
      double closestEdgeLength = 0.0;

      if (EuclidShapeTools.isFirstValueMaximum(faceXPlusDot, faceYAbsDot, -faceZPlusDot, rampDot))
      {
         firstClosestFace = Face.X_MAX;
         faceSphereCenter.setX(size.getX() - sphereOffset(firstClosestFace));
         faceSphereCenter.setZ(0.5 * size.getZ());

         edgeCandidate1.set(size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
         double candidate1_yFace = TupleTools.dot(edgeCandidate1, supportDirection);
         edgeCandidate2.set(size.getX(), 0.0, 0.0);
         double candidate2_zFace = TupleTools.dot(edgeCandidate2, supportDirection);
         edgeCandidate3.set(size.getX(), 0.0, size.getZ());
         double candidate3_rampFace = TupleTools.dot(edgeCandidate3, supportDirection);

         if (EuclidShapeTools.isFirstValueMaximum(candidate1_yFace, candidate2_zFace, candidate3_rampFace))
         {
            closestEdgeLength = size.getZ();
            closestEdgeAxis.set(Axis3D.Z);
            closestEdgeCenter.set(edgeCandidate1);
         }
         else if (candidate2_zFace > candidate3_rampFace)
         {
            closestEdgeLength = size.getY();
            closestEdgeAxis.set(Axis3D.Y);
            closestEdgeCenter.set(edgeCandidate2);
         }
         else
         {
            closestEdgeLength = size.getY();
            closestEdgeAxis.set(Axis3D.Y);
            closestEdgeCenter.set(edgeCandidate3);
         }
      }
      else if (EuclidShapeTools.isFirstValueMaximum(faceYAbsDot, -faceZPlusDot, rampDot))
      {
         firstClosestFace = isFaceYMaxCloser ? Face.Y_MAX : Face.Y_MIN;
         faceSphereCenter.setX(0.5 * size.getX());
         faceSphereCenter.setY(isFaceYMaxCloser ? 0.5 * size.getY() - sphereOffset(firstClosestFace) : -0.5 * size.getY() + sphereOffset(firstClosestFace));
         faceSphereCenter.setZ(0.5 * size.getZ());

         edgeCandidate1.set(size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
         double candidate1_xface = TupleTools.dot(edgeCandidate1, supportDirection);
         edgeCandidate2.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.0);
         double candidate2_zface = TupleTools.dot(edgeCandidate2, supportDirection);
         edgeCandidate3.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
         double candidate3_rampFace = TupleTools.dot(edgeCandidate3, supportDirection);

         if (EuclidShapeTools.isFirstValueMaximum(candidate1_xface, candidate2_zface, candidate3_rampFace))
         {
            closestEdgeLength = size.getZ();
            closestEdgeAxis.set(Axis3D.Z);
            closestEdgeCenter.set(edgeCandidate1);
         }
         else if (candidate2_zface > candidate3_rampFace)
         {
            closestEdgeLength = size.getX();
            closestEdgeAxis.set(Axis3D.X);
            closestEdgeCenter.set(edgeCandidate2);
         }
         else
         {
            closestEdgeLength = ramp3D.getRampLength();
            closestEdgeAxis.set(rampSurfaceNormal.getZ(), 0.0, -rampSurfaceNormal.getX());
            closestEdgeCenter.set(edgeCandidate3);
         }
      }
      else if (-faceZPlusDot > rampDot)
      {
         firstClosestFace = Face.Z_MIN;
         faceSphereCenter.setX(0.5 * size.getX());
         faceSphereCenter.setZ(sphereOffset(firstClosestFace));

         edgeCandidate1.set(size.getX(), 0.0, 0.0);
         double candidate1_xface = TupleTools.dot(edgeCandidate1, supportDirection);
         edgeCandidate2.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.0);
         double candidate2_yface = TupleTools.dot(edgeCandidate2, supportDirection);
         edgeCandidate3.set(0.0, 0.0, 0.0);
         double candidate3_rampFace = TupleTools.dot(edgeCandidate3, supportDirection);

         if (EuclidShapeTools.isFirstValueMaximum(candidate1_xface, candidate2_yface, candidate3_rampFace))
         {
            closestEdgeLength = size.getY();
            closestEdgeAxis.set(Axis3D.Y);
            closestEdgeCenter.set(size.getX(), 0.0, 0.0);
         }
         else if (candidate2_yface > candidate3_rampFace)
         {
            closestEdgeLength = size.getX();
            closestEdgeAxis.set(Axis3D.X);
            closestEdgeCenter.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.0);
         }
         else
         {
            closestEdgeLength = size.getY();
            closestEdgeAxis.set(Axis3D.Y);
            closestEdgeCenter.set(0.0, 0.0, 0.0);
         }
      }
      else
      {
         firstClosestFace = Face.RAMP;
         faceSphereCenter.setX(0.5 * size.getX());
         faceSphereCenter.setZ(0.5 * size.getZ());
         faceSphereCenter.scaleAdd(-sphereOffset(firstClosestFace), rampSurfaceNormal, faceSphereCenter);

         edgeCandidate1.set(size.getX(), 0.0, size.getZ());
         double candidate1_xface = TupleTools.dot(edgeCandidate1, supportDirection);
         edgeCandidate2.set(0.5 * size.getX(), isFaceYMaxCloser ? 0.5 * size.getY() : -0.5 * size.getY(), 0.5 * size.getZ());
         double candidate2_yface = TupleTools.dot(edgeCandidate2, supportDirection);
         edgeCandidate3.set(0.0, 0.0, 0.0);
         double candidate3_zface = TupleTools.dot(edgeCandidate3, supportDirection);

         if (EuclidShapeTools.isFirstValueMaximum(candidate1_xface, candidate2_yface, candidate3_zface))
         {
            closestEdgeLength = size.getY();
            closestEdgeAxis.set(Axis3D.Y);
            closestEdgeCenter.set(edgeCandidate1);
         }
         else if (candidate2_yface > candidate3_zface)
         {
            closestEdgeLength = ramp3D.getRampLength();
            closestEdgeAxis.set(rampSurfaceNormal.getZ(), 0.0, -rampSurfaceNormal.getX());
            closestEdgeCenter.set(edgeCandidate2);
         }
         else
         {
            closestEdgeLength = size.getY();
            closestEdgeAxis.set(Axis3D.Y);
            closestEdgeCenter.set(edgeCandidate3);
         }
      }

      EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);

      if (!isDirectlyAboveOrBelowFace(firstClosestFace, supportingVertexToPack))
      {
         double torusRadius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, closestEdgeLength);
         EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection, closestEdgeCenter, closestEdgeAxis, torusRadius, largeRadius, supportingVertexToPack);

         if (!isBetweenEdgeEndpoints(closestEdgeCenter, closestEdgeAxis, closestEdgeLength, supportingVertexToPack))
         {
            EuclidShapeTools.supportingVectexRamp3D(supportDirection, size, closestVertexCenter);
            EuclidShapeTools.supportingVertexSphere3D(supportDirection, closestVertexCenter, smallRadius, supportingVertexToPack);
         }
      }

      ramp3D.transformToWorld(supportingVertexToPack);

      return true;
   }

   private boolean isDirectlyAboveOrBelowFace(Face face, Point3DReadOnly query)
   {
      Vector3DReadOnly size = ramp3D.getSize();

      switch (face)
      {
         case X_MAX:
            return Math.abs(query.getY()) <= 0.5 * size.getY() && query.getZ() >= 0.0 && query.getZ() <= size.getZ() && query.getX() > size.getX();
         case Y_MIN:
         case Y_MAX:
            return isBelowRamp(query) && query.getX() <= size.getX() && query.getZ() >= 0.0;
         case Z_MIN:
            return query.getX() >= 0.0 && query.getX() <= size.getX() && Math.abs(query.getY()) <= 0.5 * size.getY();
         case RAMP:
            Vector3DReadOnly rampSurfaceNormal = ramp3D.getRampSurfaceNormal();
            double positionAlongRamp = rampSurfaceNormal.getZ() * query.getX() - rampSurfaceNormal.getX() * query.getZ();
            return positionAlongRamp >= 0.0 && positionAlongRamp <= ramp3D.getRampLength() && Math.abs(query.getY()) <= 0.5 * size.getY();
         default:
            throw new IllegalStateException();
      }
   }

   private boolean isBetweenEdgeEndpoints(Point3D edgeCenter, Vector3DReadOnly edgeAxis, double edgeLength, Point3DReadOnly query)
   {
      double percentage = EuclidGeometryTools.percentageAlongLine3D(query, edgeCenter, edgeAxis);
      double halfEdgeLength = 0.5 * edgeLength;
      return percentage >= -halfEdgeLength && percentage <= halfEdgeLength;
   }

   private boolean isBelowRamp(Point3DReadOnly query)
   {
      Vector3DReadOnly rampNormal = ramp3D.getRampSurfaceNormal();
      return query.getX() * rampNormal.getX() + rampNormal.getZ() * query.getZ() <= 0.0;
   }

   private double sphereOffset(Face face)
   {
      double a, b;
      double diagonalSquared;

      switch (face)
      {
         case RAMP:
            a = ramp3D.getRampLength();
            b = ramp3D.getSizeY();
            diagonalSquared = a * a + b * b;
            break;
         case X_MAX:
            a = ramp3D.getSizeY();
            b = ramp3D.getSizeZ();
            diagonalSquared = a * a + b * b;
            break;
         case Y_MAX:
         case Y_MIN:
            diagonalSquared = ramp3D.getRampLength() * ramp3D.getRampLength();
            break;
         case Z_MIN:
            a = ramp3D.getSizeX();
            b = ramp3D.getSizeY();
            diagonalSquared = a * a + b * b;
            break;
         default:
            throw new IllegalStateException();
      }
      double radius = largeRadius - smallRadius;
      return Math.sqrt(radius * radius - 0.25 * diagonalSquared);
   }
}
