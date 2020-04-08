package us.ihmc.euclid.visualizers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.SupportingFrameVertexHolder;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameSTPBox3D implements SupportingFrameVertexHolder
{
   private final FrameBox3DReadOnly box3D;
   private final STPBox3D stpBox3D;

   public FrameSTPBox3D(FrameBox3DReadOnly box3D)
   {
      this.box3D = box3D;
      stpBox3D = new STPBox3D(box3D);
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      return stpBox3D.getSupportingVertex(supportDirection, supportingVertexToPack);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return box3D.getReferenceFrame();
   }

}
