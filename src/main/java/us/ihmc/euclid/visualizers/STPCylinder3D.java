package us.ihmc.euclid.visualizers;

import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class STPCylinder3D implements SupportingVertexHolder
{
   private final double largeRadius = 3.00;
   private final double smallRadius = 0.05;

   private final Cylinder3DReadOnly cylinder3D;

   public STPCylinder3D(Cylinder3DReadOnly cylinder3D)
   {
      this.cylinder3D = cylinder3D;
   }

   private final Vector3D orthogonalToAxis = new Vector3D();
   private final Point3D sideSphereCenter = new Point3D();
   private final Point3D edgeSphereCenter = new Point3D();
   private final Point3D capSphereCenter = new Point3D();

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      double cylinderRadius = cylinder3D.getRadius();
      double cylinderHalfLength = cylinder3D.getHalfLength();
      UnitVector3DReadOnly cylinderAxis = cylinder3D.getAxis();
      Point3DReadOnly cylinderPosition = cylinder3D.getPosition();

      orthogonalToAxis.set(supportDirection);

      double dot = supportDirection.dot(cylinderAxis);
      double sign = dot > 0.0 ? 1.0 : -1.0;
      orthogonalToAxis.setAndScale(dot, cylinderAxis);
      orthogonalToAxis.sub(supportDirection, orthogonalToAxis);

      double distanceSquaredFromAxis = orthogonalToAxis.lengthSquared();
      double radius = largeRadius - smallRadius;

      if (distanceSquaredFromAxis < EuclidShapeTools.MIN_DISTANCE_EPSILON)
      {
         sideSphereCenter.setToNaN();
         edgeSphereCenter.setToNaN();
      }
      else
      {
         orthogonalToAxis.scale(1.0 / EuclidCoreTools.squareRoot(distanceSquaredFromAxis));

         double sideSphereRadius = Math.sqrt(radius * radius - cylinderHalfLength * cylinderHalfLength);
         sideSphereCenter.setAndScale(cylinderRadius - sideSphereRadius, orthogonalToAxis);

         edgeSphereCenter.setAndScale(cylinderRadius, orthogonalToAxis);
         edgeSphereCenter.scaleAdd(sign * cylinderHalfLength, cylinderAxis, edgeSphereCenter);
      }

      double capSphereRadius = Math.sqrt(radius * radius - cylinderRadius * cylinderRadius);
      capSphereCenter.setAndScale(sign * (cylinderHalfLength - capSphereRadius), cylinderAxis);

      if (!getSideSupportingVertex(supportDirection, supportingVertexToPack))
      {
         if (!getCapSupportingVertex(supportDirection, supportingVertexToPack))
         {
            EuclidShapeTools.supportingVertexSphere3D(supportDirection, edgeSphereCenter, smallRadius, supportingVertexToPack);
         }
      }

      supportingVertexToPack.add(cylinderPosition);

      return true;
   }

   private boolean getSideSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (sideSphereCenter.containsNaN())
         return false;
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, sideSphereCenter, largeRadius, supportingVertexToPack);
      double dot = TupleTools.dot(supportingVertexToPack, cylinder3D.getAxis());
      return dot <= cylinder3D.getHalfLength() && dot >= -cylinder3D.getHalfLength();
   }

   private final Point3D validationPoint = new Point3D();

   private boolean getCapSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, capSphereCenter, largeRadius, supportingVertexToPack);
      double dot = TupleTools.dot(supportingVertexToPack, cylinder3D.getAxis());
      validationPoint.setAndScale(dot, cylinder3D.getAxis());
      validationPoint.sub(supportingVertexToPack, validationPoint);
      return validationPoint.distanceFromOriginSquared() <= cylinder3D.getRadius() * cylinder3D.getRadius();
   }
}
