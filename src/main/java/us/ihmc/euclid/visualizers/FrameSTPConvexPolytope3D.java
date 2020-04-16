package us.ihmc.euclid.visualizers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.SupportingFrameVertexHolder;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.collision.shapeModifier.ConvexPolytope3DSTPBoundingVolume;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameSTPConvexPolytope3D implements SupportingFrameVertexHolder
{
   private FrameConvexPolytope3DReadOnly convexPolytope3D;
   private ConvexPolytope3DSTPBoundingVolume stpConvexPolytope3D;

   public FrameSTPConvexPolytope3D(FrameConvexPolytope3DReadOnly convexPolytope3D)
   {
      this.convexPolytope3D = convexPolytope3D;
      stpConvexPolytope3D = new ConvexPolytope3DSTPBoundingVolume();
      stpConvexPolytope3D.setMargins(0.005, 0.01);
      stpConvexPolytope3D.setShape3D(convexPolytope3D);
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      return stpConvexPolytope3D.getSupportingVertex(supportDirection, supportingVertexToPack);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return convexPolytope3D.getReferenceFrame();
   }

}
