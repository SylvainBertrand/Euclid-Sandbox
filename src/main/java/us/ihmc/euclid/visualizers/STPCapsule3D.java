package us.ihmc.euclid.visualizers;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class STPCapsule3D implements SupportingVertexHolder
{
   private final double largeRadius = 5.0;
   private final double smallRadius = 0.0;

   private final Capsule3DReadOnly capsule3D;

   public STPCapsule3D(Capsule3DReadOnly capsule3D)
   {
      this.capsule3D = capsule3D;
   }

   private final Vector3D orthogonalToAxis = new Vector3D();
   private final Point3D sideSphereCenter = new Point3D();
   private final Point3D edgeSphereCenter = new Point3D();
   private final Point3D capSphereCenter = new Point3D();

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      double capsuleRadius = capsule3D.getRadius();
      double capsuleHalfLength = capsule3D.getHalfLength();
      UnitVector3DReadOnly capsuleAxis = capsule3D.getAxis();
      Point3DReadOnly capsulePosition = capsule3D.getPosition();

      orthogonalToAxis.set(supportDirection);

      double dot = supportDirection.dot(capsuleAxis);
      double sign = dot > 0.0 ? 1.0 : -1.0;
      orthogonalToAxis.setAndScale(dot, capsuleAxis);
      orthogonalToAxis.sub(supportDirection, orthogonalToAxis);
      edgeSphereCenter.setAndScale(sign * capsuleHalfLength, capsuleAxis);

      double distanceSquaredFromAxis = orthogonalToAxis.lengthSquared();

      if (distanceSquaredFromAxis < EuclidShapeTools.MIN_DISTANCE_EPSILON)
      {
         sideSphereCenter.setToNaN();
         edgeSphereCenter.setToNaN();
      }
      else
      {
         orthogonalToAxis.scale(1.0 / EuclidCoreTools.squareRoot(distanceSquaredFromAxis));

         double sideSphereRadius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, capsule3D.getLength());
         sideSphereCenter.setAndScale(capsuleRadius - sideSphereRadius, orthogonalToAxis);

      }

      double capSphereRadius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, 2.0 * capsuleRadius);
      capSphereCenter.setAndScale(sign * (capsuleHalfLength - capSphereRadius), capsuleAxis);

      if (!getSideSupportingVertex(supportDirection, supportingVertexToPack))
      {
         EuclidShapeTools.supportingVertexSphere3D(supportDirection, edgeSphereCenter, capsuleRadius + smallRadius, supportingVertexToPack);
      }

      supportingVertexToPack.add(capsulePosition);

      return true;
   }

   private boolean getSideSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (sideSphereCenter.containsNaN())
         return false;
      EuclidShapeTools.supportingVertexSphere3D(supportDirection, sideSphereCenter, largeRadius, supportingVertexToPack);
      double dot = TupleTools.dot(supportingVertexToPack, capsule3D.getAxis());
      return dot <= capsule3D.getHalfLength() && dot >= -capsule3D.getHalfLength();
   }
}
