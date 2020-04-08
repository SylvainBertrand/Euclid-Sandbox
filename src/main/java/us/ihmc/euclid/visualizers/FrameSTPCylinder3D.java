package us.ihmc.euclid.visualizers;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.SupportingFrameVertexHolder;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameSTPCylinder3D implements SupportingFrameVertexHolder
{
   private final FrameCylinder3DReadOnly cylinder3D;
   private final STPCylinder3D stpCylinder3D;

   public FrameSTPCylinder3D(FrameCylinder3DReadOnly cylinder3D)
   {
      this.cylinder3D = cylinder3D;
      stpCylinder3D = new STPCylinder3D(cylinder3D);
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      return stpCylinder3D.getSupportingVertex(supportDirection, supportingVertexToPack);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return cylinder3D.getReferenceFrame();
   }

}
