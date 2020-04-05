package us.ihmc.euclid.visualizers;

import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;

public class Shape3DMeshFactories
{
   public static Node toFrameShape3DMesh(FrameShape3DBasics shape3D, Color color)
   {
      FrameShape3DBasics copy = shape3D.copy();
      copy.changeFrame(ReferenceFrame.getWorldFrame());
      return toShape3DMesh(copy, color);
   }

   public static Node toShape3DMesh(Shape3DReadOnly shape3D, Color color)
   {
      if (shape3D instanceof PointShape3DReadOnly)
         return Shape3DMeshFactories.toPointShape3DMesh((PointShape3DReadOnly) shape3D, color);
      if (shape3D instanceof Sphere3DReadOnly)
         return Shape3DMeshFactories.toSphere3DMesh((Sphere3DReadOnly) shape3D, color);
      if (shape3D instanceof Box3DReadOnly)
         return Shape3DMeshFactories.toBox3DMesh((Box3DReadOnly) shape3D, color);
      if (shape3D instanceof Ramp3DReadOnly)
         return Shape3DMeshFactories.toRamp3DMesh((Ramp3DReadOnly) shape3D, color);
      if (shape3D instanceof Ellipsoid3DReadOnly)
         return Shape3DMeshFactories.toEllipsoid3DMesh((Ellipsoid3DReadOnly) shape3D, color);
      if (shape3D instanceof Capsule3DReadOnly)
         return Shape3DMeshFactories.toCapsule3DMesh((Capsule3DReadOnly) shape3D, color);
      throw new UnsupportedOperationException("Unsupported shape " + shape3D);
   }

   public static Node toPointShape3DMesh(PointShape3DReadOnly pointShape3D, Color color)
   {
      return Shape3DMeshFactories.togeneratePointMesh(pointShape3D, color, 0.01);
   }

   public static Node toSphere3DMesh(Sphere3DReadOnly sphere3D, Color color)
   {
      return Shape3DMeshFactories.togeneratePointMesh(sphere3D.getPosition(), color, sphere3D.getRadius());
   }

   public static Node toBox3DMesh(Box3DReadOnly box3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Cube(box3D.getSizeX(), box3D.getSizeY(), box3D.getSizeZ(), true);
      mesh = MeshDataHolder.rotate(mesh, new AxisAngle(box3D.getOrientation()));
      mesh = MeshDataHolder.translate(mesh, box3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node toCapsule3DMesh(Capsule3DReadOnly capsule3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Capsule(capsule3D.getLength(), capsule3D.getRadius(), capsule3D.getRadius(), capsule3D.getRadius(), 64, 64);
      mesh = MeshDataHolder.rotate(mesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(capsule3D.getAxis()));
      mesh = MeshDataHolder.translate(mesh, capsule3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node toRamp3DMesh(Ramp3DReadOnly ramp3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Wedge(ramp3D.getSizeX(), ramp3D.getSizeY(), ramp3D.getSizeZ());
      mesh = MeshDataHolder.translate(mesh, (float) (0.5 * ramp3D.getSizeX()), 0.0f, 0.0f);
      mesh = MeshDataHolder.rotate(mesh, new AxisAngle(ramp3D.getOrientation()));
      mesh = MeshDataHolder.translate(mesh, ramp3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node toEllipsoid3DMesh(Ellipsoid3DReadOnly ellipsoid3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Ellipsoid(ellipsoid3D.getRadiusX(), ellipsoid3D.getRadiusY(), ellipsoid3D.getRadiusZ(), 128, 128);
      mesh = MeshDataHolder.rotate(mesh, new AxisAngle(ellipsoid3D.getOrientation()));
      mesh = MeshDataHolder.translate(mesh, ellipsoid3D.getPosition());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node togenerateCylinder3DMesh(Cylinder3DReadOnly cylinder3D, Color color)
   {
      MeshDataHolder mesh = MeshDataGenerator.Cylinder(cylinder3D.getRadius(), cylinder3D.getLength(), 128);
      mesh = MeshDataHolder.rotate(mesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(cylinder3D.getAxis()));
      mesh = MeshDataHolder.translate(mesh, cylinder3D.getBottomCenter());
      MeshView meshView = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(mesh));
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static <F extends Face3DReadOnly> Node toFace3DsMesh(List<F> faces)
   {
      Group group = new Group();

      for (F face : faces)
      {
         List<Point3D> cwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         cwFaceVertices.add(cwFaceVertices.get(0));
         cwFaceVertices.add(0, new Point3D(face.getCentroid()));
         List<Point3D> ccwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         Collections.reverse(ccwFaceVertices);
         ccwFaceVertices.add(ccwFaceVertices.get(0));
         ccwFaceVertices.add(0, new Point3D(face.getCentroid()));

         MeshView outsideFaceNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Polygon(ccwFaceVertices)));
         outsideFaceNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + faces.indexOf(face) + ": " + face.toString()));
         double hue = EuclidCoreRandomTools.nextDouble(new Random(face.hashCode()), 0.0, 360.0);
         outsideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 0.5)));
         group.getChildren().add(outsideFaceNode);

         MeshView insideFaceNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Polygon(cwFaceVertices)));
         insideFaceNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + faces.indexOf(face) + ": " + face.toString()));
         insideFaceNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 1.0).darker()));
         //         group.getChildren().add(insideFaceNode);
      }

      return group;
   }

   public static Node generateFace3DsMesh(List<? extends Face3DReadOnly> faces, Color color)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      for (Face3DReadOnly face : faces)
      {
         List<Point3D> ccwFaceVertices = face.getVertices().stream().map(Point3D::new).collect(Collectors.toList());
         Collections.reverse(ccwFaceVertices);
         meshBuilder.addMesh(MeshDataGenerator.Polygon(ccwFaceVertices));
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(color));
      return meshView;
   }

   public static Node toFace3DsNormalMesh(List<? extends Face3DReadOnly> faces)
   {
      Group group = new Group();

      for (int i = 0; i < faces.size(); i++)
      {
         Face3DReadOnly face = faces.get(i);
         double scale = Math.max(0.00003, face.getEdges().stream().mapToDouble(HalfEdge3DReadOnly::length).max().getAsDouble());
         double height = 0.010 * scale;
         double radius = 0.005 * scale;
         AxisAngle orientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(face.getNormal());
         double hue = EuclidCoreRandomTools.nextDouble(new Random(face.hashCode()), 0.0, 360.0);
         MeshDataHolder cone = MeshDataGenerator.Cone(height, radius, 32);
         cone = MeshDataHolder.rotate(cone, orientation);
         cone = MeshDataHolder.translate(cone, face.getCentroid());
         MeshView normalNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(cone));
         normalNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + faces.indexOf(face) + ": " + face.toString()));
         normalNode.setMaterial(new PhongMaterial(Color.hsb(hue, 0.9, 0.9, 1.0)));
         group.getChildren().add(normalNode);
      }

      return group;
   }

   public static Node togeneratePointMesh(Tuple3DReadOnly position, Color color, double size)
   {
      MeshDataHolder sphereMeshData = MeshDataHolder.translate(MeshDataGenerator.Sphere(size, 64, 64), position);
      MeshView node = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(sphereMeshData));
      node.setMaterial(new PhongMaterial(color));
      return node;
   }

   public static Node toHalfEdge3DsMesh(List<? extends HalfEdge3DReadOnly> edges, Color color, double width)
   {
      Group group = new Group();

      if (edges == null)
         return group;

      for (HalfEdge3DReadOnly edge : edges)
      {
         MeshDataHolder line = MeshDataGenerator.Line(edge.getOrigin(), edge.getDestination(), width);
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(line));
         edgeNode.setMaterial(new PhongMaterial(color));
         edgeNode.addEventHandler(MouseEvent.MOUSE_CLICKED, e -> System.out.println("Index: " + edges.indexOf(edge) + ": " + edge.toString()));
         group.getChildren().add(edgeNode);
      }

      return group;
   }

   public static Node toMultilineMesh(List<? extends Point3DReadOnly> multiline, boolean close, Color color, double width)
   {
      Group group = new Group();

      if (multiline == null)
         return group;

      if (multiline.size() < 2)
         return group;

      for (int i = 1; i < multiline.size(); i++)
      {
         Point3DReadOnly start = multiline.get(i - 1);
         Point3DReadOnly end = multiline.get(i);
         MeshDataHolder line = MeshDataGenerator.Line(start, end, width);
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(line));
         edgeNode.setMaterial(new PhongMaterial(color));
         group.getChildren().add(edgeNode);
         edgeNode.addEventHandler(MouseEvent.MOUSE_CLICKED,
                                  e -> System.out.println("start: " + EuclidCoreIOTools.getTuple3DString(start) + ", end: "
                                        + EuclidCoreIOTools.getTuple3DString(end)));
      }

      if (close)
      {
         Point3DReadOnly start = multiline.get(multiline.size() - 1);
         Point3DReadOnly end = multiline.get(0);
         MeshDataHolder line = MeshDataGenerator.Line(start, end, width);
         MeshView edgeNode = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(line));
         edgeNode.setMaterial(new PhongMaterial(color));
         group.getChildren().add(edgeNode);
         edgeNode.addEventHandler(MouseEvent.MOUSE_CLICKED,
                                  e -> System.out.println("start: " + EuclidCoreIOTools.getTuple3DString(start) + ", end: "
                                        + EuclidCoreIOTools.getTuple3DString(end)));
      }

      return group;
   }

   public static Color nextColor(Random random)
   {
      return Color.hsb(EuclidCoreRandomTools.nextDouble(random, 0.0, 360.0), 0.9, 0.9);
   }
}
