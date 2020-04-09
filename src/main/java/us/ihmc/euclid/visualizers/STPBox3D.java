package us.ihmc.euclid.visualizers;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.triangleIsoscelesHeight;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class STPBox3D implements SupportingVertexHolder
{
   private final double largeRadius = 15.00;
   private final double smallRadius = 0.005;

   private final Box3DReadOnly box3D;
   private final Vector3D halfSize = new Vector3D();
   private final Vector3D supportDirectionLocal = new Vector3D();
   private final Point3D faceSphereCenter = new Point3D();
   private final Point3D closestEdgeCenter = new Point3D();
   private final Point3D closestVertexCenter = new Point3D();

   public STPBox3D(Box3DReadOnly box3D)
   {
      this.box3D = box3D;
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (box3D.getPose().hasRotation())
      {
         box3D.getPose().inverseTransform(supportDirection, supportDirectionLocal);
         supportDirection = supportDirectionLocal;
      }

      halfSize.setAndScale(0.5, box3D.getSize());

      double faceXPlusDot = supportDirection.getX();
      double faceYPlusDot = supportDirection.getY();
      double faceZPlusDot = supportDirection.getZ();

      double faceXAbsDot = Math.abs(faceXPlusDot);
      double faceYAbsDot = Math.abs(faceYPlusDot);
      double faceZAbsDot = Math.abs(faceZPlusDot);

      Axis3D firstClosestFace;
      double closestEdgeLength;
      faceSphereCenter.setToZero();
      closestEdgeCenter.setToZero();

      boolean isFaceXMaxCloser = faceXPlusDot > 0.0;
      boolean isFaceYMaxCloser = faceYPlusDot > 0.0;
      boolean isFaceZMaxCloser = faceZPlusDot > 0.0;

      closestVertexCenter.set(isFaceXMaxCloser ? halfSize.getX() : -halfSize.getX(),
                              isFaceYMaxCloser ? halfSize.getY() : -halfSize.getY(),
                              isFaceZMaxCloser ? halfSize.getZ() : -halfSize.getZ());

      Axis3D closestEdgeTorusAxis;

      if (faceXAbsDot > faceYAbsDot)
      {
         if (faceXAbsDot > faceZAbsDot)
         { // Closest is one of the 2 x-faces
            firstClosestFace = Axis3D.X;
            faceSphereCenter.setX(isFaceXMaxCloser ? halfSize.getX() - sphereOffset(firstClosestFace) : -halfSize.getX() + sphereOffset(firstClosestFace));
            closestEdgeCenter.setX(isFaceXMaxCloser ? halfSize.getX() : -halfSize.getX());

            if (faceYAbsDot > faceZAbsDot)
            { // 2nd closest face is one of the 2 y-faces
               closestEdgeLength = box3D.getSizeZ();
               closestEdgeTorusAxis = Axis3D.Z;
               closestEdgeCenter.setY(isFaceYMaxCloser ? halfSize.getY() : -halfSize.getY());
               closestVertexCenter.set(closestEdgeCenter);
               closestVertexCenter.setZ(isFaceZMaxCloser ? halfSize.getZ() : -halfSize.getZ());
            }
            else
            { // 2nd closest face is one of the 2 z-faces
               closestEdgeLength = box3D.getSizeY();
               closestEdgeTorusAxis = Axis3D.Y;
               closestEdgeCenter.setZ(isFaceZMaxCloser ? halfSize.getZ() : -halfSize.getZ());
            }
         }
         else
         { // Closest is one of the 2 z-faces
            firstClosestFace = Axis3D.Z;
            faceSphereCenter.setZ(isFaceZMaxCloser ? halfSize.getZ() - sphereOffset(firstClosestFace) : -halfSize.getZ() + sphereOffset(firstClosestFace));
            closestEdgeCenter.setZ(isFaceZMaxCloser ? halfSize.getZ() : -halfSize.getZ());

            closestEdgeLength = box3D.getSizeY();
            closestEdgeTorusAxis = Axis3D.Y;
            closestEdgeCenter.setX(isFaceXMaxCloser ? halfSize.getX() : -halfSize.getX());
         }
      }
      else if (faceYAbsDot > faceZAbsDot)
      { // Closest is one of the 2 y-faces
         firstClosestFace = Axis3D.Y;
         faceSphereCenter.setY(isFaceYMaxCloser ? halfSize.getY() - sphereOffset(firstClosestFace) : -halfSize.getY() + sphereOffset(firstClosestFace));
         closestEdgeCenter.setY(isFaceYMaxCloser ? halfSize.getY() : -halfSize.getY());

         if (faceXAbsDot > faceZAbsDot)
         { // 2nd closest face is one of the 2 x-faces
            closestEdgeLength = box3D.getSizeZ();
            closestEdgeTorusAxis = Axis3D.Z;
            closestEdgeCenter.setX(isFaceXMaxCloser ? halfSize.getX() : -halfSize.getX());
         }
         else
         { // 2nd closest face is one of the 2 z-faces
            closestEdgeLength = box3D.getSizeX();
            closestEdgeTorusAxis = Axis3D.X;
            closestEdgeCenter.setZ(isFaceZMaxCloser ? halfSize.getZ() : -halfSize.getZ());
         }
      }
      else
      { // Closest is one of the 2 z-faces
         firstClosestFace = Axis3D.Z;
         faceSphereCenter.setZ(isFaceZMaxCloser ? halfSize.getZ() - sphereOffset(firstClosestFace) : -halfSize.getZ() + sphereOffset(firstClosestFace));
         closestEdgeCenter.setZ(isFaceZMaxCloser ? halfSize.getZ() : -halfSize.getZ());

         closestEdgeLength = box3D.getSizeX();
         closestEdgeTorusAxis = Axis3D.X;
         closestEdgeCenter.setY(isFaceYMaxCloser ? halfSize.getY() : -halfSize.getY());
      }

      EuclidShapeTools.supportingVertexSphere3D(supportDirection, faceSphereCenter, largeRadius, supportingVertexToPack);

      if (!isDirectlyAboveOrBelowFace(firstClosestFace, supportingVertexToPack))
      {
         EuclidShapeTools.innerSupportingVertexTorus3D(supportDirection,
                                                       closestEdgeCenter,
                                                       closestEdgeTorusAxis,
                                                       triangleIsoscelesHeight(largeRadius - smallRadius, closestEdgeLength),
                                                       largeRadius,
                                                       supportingVertexToPack);

         if (Math.abs(Axis3D.get(supportingVertexToPack, closestEdgeTorusAxis)) > 0.5 * closestEdgeLength)
         {
            EuclidShapeTools.supportingVertexSphere3D(supportDirection, closestVertexCenter, smallRadius, supportingVertexToPack);
         }
      }

      box3D.transformToWorld(supportingVertexToPack);

      return true;
   }

   private boolean isDirectlyAboveOrBelowFace(Axis3D face, Point3DReadOnly query)
   {
      switch (face)
      {
         case X:
            return Math.abs(query.getY()) <= halfSize.getY() && Math.abs(query.getZ()) <= halfSize.getZ();
         case Y:
            return Math.abs(query.getX()) <= halfSize.getX() && Math.abs(query.getZ()) <= halfSize.getZ();
         case Z:
            return Math.abs(query.getX()) <= halfSize.getX() && Math.abs(query.getY()) <= halfSize.getY();
         default:
            throw new IllegalStateException();
      }
   }

   private double sphereOffset(Axis3D face)
   {
      double a = Axis3D.get(box3D.getSize(), face.getNextClockwiseAxis());
      double b = Axis3D.get(box3D.getSize(), face.getNextCounterClockwiseAxis());
      double diagonalSquared = a * a + b * b;
      double radius = largeRadius - smallRadius;
      return Math.sqrt(radius * radius - 0.25 * diagonalSquared);
   }
}
