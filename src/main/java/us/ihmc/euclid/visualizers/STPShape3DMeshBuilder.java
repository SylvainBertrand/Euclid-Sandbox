package us.ihmc.euclid.visualizers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleFunction;

import gnu.trove.list.array.TIntArrayList;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.collision.shapeModifier.Box3DSTPBoundingVolume;
import us.ihmc.euclid.shape.collision.shapeModifier.Capsule3DSTPBoundingVolume;
import us.ihmc.euclid.shape.collision.shapeModifier.ConvexPolytope3DSTPBoundingVolume;
import us.ihmc.euclid.shape.collision.shapeModifier.Cylinder3DSTPBoundingVolume;
import us.ihmc.euclid.shape.collision.shapeModifier.Ramp3DSTPBoundingVolume;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.RampPolytope3DView;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;

public class STPShape3DMeshBuilder
{
   private static final double TwoPi = 2.0 * Math.PI;

   public static Node toSTPBox3DMesh(Box3DSTPBoundingVolume stpBox3D)
   {
      BoxPolytope3DView boxPolytope = stpBox3D.getShape3D().asConvexPolytope();
      double largeRadius = stpBox3D.getLargeRadius();
      double smallRadius = stpBox3D.getSmallRadius();

      return toSTPConvexPolytope3DMesh(boxPolytope, largeRadius, smallRadius);
   }

   public static Node toSTPConvexPolytope3DMesh(ConvexPolytope3DSTPBoundingVolume stpConvexPolytope3D)
   {
      ConvexPolytope3DReadOnly convexPolytope = stpConvexPolytope3D.getShape3D();
      double largeRadius = stpConvexPolytope3D.getLargeRadius();
      double smallRadius = stpConvexPolytope3D.getSmallRadius();

      return toSTPConvexPolytope3DMesh(convexPolytope, largeRadius, smallRadius);
   }

   public static Node toSTPCapsule3DMesh(Capsule3DSTPBoundingVolume stpCapsule3D)
   {
      Capsule3DReadOnly capsule = stpCapsule3D.getShape3D();
      double largeRadius = stpCapsule3D.getLargeRadius();
      double smallRadius = stpCapsule3D.getSmallRadius();

      double length = capsule.getLength();
      UnitVector3DReadOnly axis = capsule.getAxis();
      Point3DReadOnly position = capsule.getPosition();
      Point3DReadOnly topCenter = capsule.getTopCenter();
      Point3DReadOnly bottomCenter = capsule.getBottomCenter();

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();

      // Side face
      Vector3D axisOrthogonal = newOrthogonalVector(axis);
      double sphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, length);
      Point3D sphereCenter = new Point3D();
      sphereCenter.scaleAdd(-sphereOffset, axisOrthogonal, position);

      UnitVector3D startDirection = new UnitVector3D();
      UnitVector3D endDirection = new UnitVector3D();

      startDirection.sub(bottomCenter, sphereCenter);
      endDirection.sub(topCenter, sphereCenter);

      MeshDataHolder arc = toArcPointsAndNormals(sphereCenter, largeRadius, startDirection, endDirection, 64);
      meshBuilder.addMesh(applyRevolution(arc, position, axis, 0.0, TwoPi, 64, false), Color.CORNFLOWERBLUE);

      double limitPositionOnAxis = 0.5 * length * largeRadius / (largeRadius - smallRadius);
      double limitRadius = sphereOffset * smallRadius / (largeRadius - smallRadius);
      MeshDataHolder sideLimitMesh = MeshDataGenerator.ArcTorus(0.0, TwoPi, limitRadius, 0.001, 64);
      sideLimitMesh = MeshDataHolder.rotate(sideLimitMesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(axis));
      sideLimitMesh = MeshDataHolder.translate(sideLimitMesh, position);
      Arrays.asList(sideLimitMesh.getVertices()).forEach(v -> v.scaleAdd(limitPositionOnAxis, axis, v));
      meshBuilder.addMesh(sideLimitMesh, Color.CORNFLOWERBLUE);

      Arrays.asList(sideLimitMesh.getVertices()).forEach(v -> v.scaleAdd(-2.0 * limitPositionOnAxis, axis, v));
      meshBuilder.addMesh(sideLimitMesh, Color.CORNFLOWERBLUE);

      // Cap faces
      startDirection.set(axis);

      arc = toArcPointsAndNormals(topCenter, smallRadius, endDirection, startDirection, 64);
      MeshDataHolder capMesh = applyRevolution(arc, position, axis, 0.0, TwoPi, 64, false);
      meshBuilder.addMesh(capMesh, Color.CORNFLOWERBLUE);

      // Flipping the meshes around to draw the bottom cap
      RotationMatrix flipRotation = new RotationMatrix();
      flipRotation.setAxisAngle(axisOrthogonal.getX(), axisOrthogonal.getY(), axisOrthogonal.getZ(), Math.PI);
      Arrays.asList(capMesh.getVertices()).forEach(v ->
      {
         v.sub(position);
         flipRotation.transform(v);
         v.add(position);
      });
      Arrays.asList(capMesh.getVertexNormals()).forEach(n -> flipRotation.transform(n));
      meshBuilder.addMesh(capMesh, Color.CORNFLOWERBLUE);

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }

   public static Node toSTPCylinder3DMesh(Cylinder3DSTPBoundingVolume stpCylinder3D)
   {
      Cylinder3DReadOnly cylinder = stpCylinder3D.getShape3D();
      double largeRadius = stpCylinder3D.getLargeRadius();
      double smallRadius = stpCylinder3D.getSmallRadius();

      double length = cylinder.getLength();
      double radius = cylinder.getRadius();
      UnitVector3DReadOnly axis = cylinder.getAxis();
      Point3DReadOnly position = cylinder.getPosition();
      Point3DReadOnly topCenter = cylinder.getTopCenter();
      Point3DReadOnly bottomCenter = cylinder.getBottomCenter();

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();

      { // Side face
         Vector3D axisOrthogonal = newOrthogonalVector(axis);
         double sphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, length);
         Point3D sphereCenter = new Point3D();
         sphereCenter.scaleAdd(-sphereOffset + radius, axisOrthogonal, position);

         UnitVector3D startDirection = new UnitVector3D();
         UnitVector3D endDirection = new UnitVector3D();

         startDirection.scaleAdd(radius, axisOrthogonal, bottomCenter);
         startDirection.sub(sphereCenter);
         endDirection.scaleAdd(radius, axisOrthogonal, topCenter);
         endDirection.sub(sphereCenter);

         MeshDataHolder arc = toArcPointsAndNormals(sphereCenter, largeRadius, startDirection, endDirection, 64);
         meshBuilder.addMesh(applyRevolution(arc, position, axis, 0.0, TwoPi, 64, false), Color.CORNFLOWERBLUE);

         double limitPositionOnAxis = 0.5 * length * largeRadius / (largeRadius - smallRadius);
         double limitRadius = radius + sphereOffset * smallRadius / (largeRadius - smallRadius);
         MeshDataHolder sideLimitMesh = MeshDataGenerator.ArcTorus(0.0, TwoPi, limitRadius, 0.001, 64);
         sideLimitMesh = MeshDataHolder.rotate(sideLimitMesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(axis));
         sideLimitMesh = MeshDataHolder.translate(sideLimitMesh, position);
         Arrays.asList(sideLimitMesh.getVertices()).forEach(v -> v.scaleAdd(limitPositionOnAxis, axis, v));
         meshBuilder.addMesh(sideLimitMesh, Color.CORNFLOWERBLUE);

         Arrays.asList(sideLimitMesh.getVertices()).forEach(v -> v.scaleAdd(-2.0 * limitPositionOnAxis, axis, v));
         meshBuilder.addMesh(sideLimitMesh, Color.CORNFLOWERBLUE);
      }

      { // Cap faces
         double sphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, 2.0 * radius);
         Point3D sphereCenter = new Point3D();
         sphereCenter.scaleAdd(-sphereOffset, axis, topCenter);

         Vector3D axisOrthogonal = newOrthogonalVector(axis);

         UnitVector3D boundaryDirection = new UnitVector3D();
         boundaryDirection.scaleAdd(radius, axisOrthogonal, topCenter);
         boundaryDirection.sub(sphereCenter);

         MeshDataHolder arc = toArcPointsAndNormals(sphereCenter, largeRadius, boundaryDirection, axis, 64);
         MeshDataHolder capMesh = applyRevolution(arc, position, axis, 0.0, TwoPi, 64, false);
         meshBuilder.addMesh(capMesh, Color.CORNFLOWERBLUE);

         double limitPositionOnAxis = 0.5 * length + sphereOffset * smallRadius / (largeRadius - smallRadius);
         double limitRadius = radius * largeRadius / (largeRadius - smallRadius);
         MeshDataHolder capLimitMesh = MeshDataGenerator.ArcTorus(0.0, TwoPi, limitRadius, 0.001, 64);
         capLimitMesh = MeshDataHolder.rotate(capLimitMesh, EuclidGeometryTools.axisAngleFromZUpToVector3D(axis));
         capLimitMesh = MeshDataHolder.translate(capLimitMesh, position);
         Arrays.asList(capLimitMesh.getVertices()).forEach(v -> v.scaleAdd(limitPositionOnAxis, axis, v));
         meshBuilder.addMesh(capLimitMesh, Color.CORNFLOWERBLUE);

         // Flipping the meshes around to draw the bottom cap
         RotationMatrix flipRotation = new RotationMatrix();
         flipRotation.setAxisAngle(axisOrthogonal.getX(), axisOrthogonal.getY(), axisOrthogonal.getZ(), Math.PI);
         Arrays.asList(capMesh.getVertices()).forEach(v ->
         {
            v.sub(position);
            flipRotation.transform(v);
            v.add(position);
         });
         Arrays.asList(capMesh.getVertexNormals()).forEach(n -> flipRotation.transform(n));
         meshBuilder.addMesh(capMesh, Color.CORNFLOWERBLUE);

         Arrays.asList(capLimitMesh.getVertices()).forEach(v -> v.scaleAdd(-2.0 * limitPositionOnAxis, axis, v));
         meshBuilder.addMesh(capLimitMesh, Color.CORNFLOWERBLUE);
      }

      { // Edges
         Vector3D axisOrthogonal = newOrthogonalVector(axis);
         Point3D arcCenter = new Point3D();
         arcCenter.scaleAdd(radius, axisOrthogonal, topCenter);

         double capSphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, 2.0 * radius);
         Point3D capSphereCenter = new Point3D();
         capSphereCenter.scaleAdd(-capSphereOffset, axis, topCenter);

         double sideSphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, length);
         Point3D sideSphereCenter = new Point3D();
         sideSphereCenter.scaleAdd(-sideSphereOffset + radius, axisOrthogonal, position);

         UnitVector3D startDirection = new UnitVector3D();
         UnitVector3D endDirection = new UnitVector3D();

         startDirection.sub(arcCenter, sideSphereCenter);
         endDirection.sub(arcCenter, capSphereCenter);

         MeshDataHolder arc = toArcPointsAndNormals(arcCenter, smallRadius, startDirection, endDirection, 32);
         MeshDataHolder edgeMesh = applyRevolution(arc, position, axis, 0.0, TwoPi, 64, false);
         meshBuilder.addMesh(edgeMesh, Color.BLUEVIOLET);

         RotationMatrix flipRotation = new RotationMatrix();
         flipRotation.setAxisAngle(axisOrthogonal.getX(), axisOrthogonal.getY(), axisOrthogonal.getZ(), Math.PI);
         Arrays.asList(edgeMesh.getVertices()).forEach(v ->
         {
            v.sub(position);
            flipRotation.transform(v);
            v.add(position);
         });
         Arrays.asList(edgeMesh.getVertexNormals()).forEach(n -> flipRotation.transform(n));
         meshBuilder.addMesh(edgeMesh, Color.BLUEVIOLET);
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }

   public static Node toSTPRamp3DMesh(Ramp3DSTPBoundingVolume stpRamp3D)
   {
      RampPolytope3DView rampPolytope = stpRamp3D.getShape3D().asConvexPolytope();
      double largeRadius = stpRamp3D.getLargeRadius();
      double smallRadius = stpRamp3D.getSmallRadius();

      return toSTPConvexPolytope3DMesh(rampPolytope, largeRadius, smallRadius);
   }

   private static Node toSTPConvexPolytope3DMesh(ConvexPolytope3DReadOnly convexPolytope, double largeRadius, double smallRadius)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();

      for (int faceIndex = 0; faceIndex < convexPolytope.getNumberOfFaces(); faceIndex++)
      { // Produce faces' big sphere mesh
         List<MeshDataHolder> meshes = toFaceSpheres(convexPolytope.getFace(faceIndex), largeRadius, smallRadius, true);
         meshes.forEach(mesh -> meshBuilder.addMesh(mesh, Color.CORNFLOWERBLUE));
      }

      Set<HalfEdge3DReadOnly> processedHalfEdgeSet = new HashSet<>();

      for (int edgeIndex = 0; edgeIndex < convexPolytope.getNumberOfHalfEdges(); edgeIndex++)
      { // Building the edges' torus
         HalfEdge3DReadOnly halfEdge = convexPolytope.getHalfEdge(edgeIndex);
         if (processedHalfEdgeSet.contains(halfEdge.getTwin()))
            continue;

         processedHalfEdgeSet.add(halfEdge);
         meshBuilder.addMesh(toHalfEdgeTorus(halfEdge, largeRadius, smallRadius), Color.BLUEVIOLET);
      }

      for (int vertexIndex = 0; vertexIndex < convexPolytope.getNumberOfVertices(); vertexIndex++)
      { // Building the vertices' small sphere
         Vertex3DReadOnly vertex = convexPolytope.getVertex(vertexIndex);
         List<MeshDataHolder> meshes = toVertexSphere(vertex, largeRadius, smallRadius, false);
         meshes.forEach(mesh -> meshBuilder.addMesh(mesh, Color.ORANGE));
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }

   public static List<MeshDataHolder> toBoxFaceSphere(Face3DReadOnly face, double largeRadius, double smallRadius, boolean highlightLimits)
   {
      List<MeshDataHolder> meshes = new ArrayList<>();

      Point3D sphereCenter = new Point3D();
      Vector3D limitA = new Vector3D();
      Vector3D limitB = new Vector3D();
      Vector3D limitC = new Vector3D();
      Vector3D limitD = new Vector3D();

      double centerOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, face.getVertex(0).distance(face.getVertex(2)));
      sphereCenter.scaleAdd(-centerOffset, face.getNormal(), face.getCentroid());

      limitA.sub(face.getVertex(0), sphereCenter);
      limitB.sub(face.getVertex(1), sphereCenter);
      limitC.sub(face.getVertex(2), sphereCenter);
      limitD.sub(face.getVertex(3), sphereCenter);

      meshes.add(toPartialSphereMesh(sphereCenter, limitA, limitB, limitC, largeRadius, 32));
      meshes.add(toPartialSphereMesh(sphereCenter, limitA, limitC, limitD, largeRadius, 32));

      if (highlightLimits)
         meshes.addAll(toCyclicFaceSphereLimits(face, largeRadius, smallRadius, 0.001));

      return meshes;
   }

   public static List<MeshDataHolder> toFaceSpheres(Face3DReadOnly face, double largeRadius, double smallRadius, boolean highlightLimits)
   {
      List<MeshDataHolder> meshes = new ArrayList<>();
      boolean isFaceCyclicPolygon = isFaceCyclicPolygon(face, largeRadius, smallRadius);

      for (int vertexIndex = 1; vertexIndex < face.getNumberOfEdges() - 1; vertexIndex++)
      {
         Vertex3DReadOnly v0 = face.getVertex(0);
         Vertex3DReadOnly v1 = face.getVertex(vertexIndex);
         Vertex3DReadOnly v2 = face.getVertex(vertexIndex + 1);

         meshes.addAll(toFaceSubSphere(face, v0, v1, v2, largeRadius, smallRadius, highlightLimits && !isFaceCyclicPolygon));
      }

      if (highlightLimits && isFaceCyclicPolygon)
      {
         meshes.addAll(toCyclicFaceSphereLimits(face, largeRadius, smallRadius, 0.001));
      }

      return meshes;
   }

   private static boolean isFaceCyclicPolygon(Face3DReadOnly face, double largeRadius, double smallRadius)
   {
      if (face.getNumberOfEdges() <= 3)
         return true;

      Point3D sphereCenter = new Point3D();
      Vertex3DReadOnly v0 = face.getVertex(0);
      Vertex3DReadOnly v1 = face.getVertex(1);
      Vertex3DReadOnly v2 = face.getVertex(2);
      double radius = largeRadius - smallRadius;
      EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1, v2, radius, sphereCenter);
      double radiusSquared = EuclidCoreTools.square(radius);

      for (int vertexIndex = 3; vertexIndex < face.getNumberOfEdges(); vertexIndex++)
      {
         double distanceSquared = face.getVertex(vertexIndex).distanceSquared(sphereCenter);
         if (!EuclidCoreTools.epsilonEquals(radiusSquared, distanceSquared, 1.0e-12))
            return false;
      }

      return true;
   }

   public static List<MeshDataHolder> toFaceSubSphere(Face3DReadOnly owner, Vertex3DReadOnly v0, Vertex3DReadOnly v1, Vertex3DReadOnly v2, double largeRadius,
                                                      double smallRadius, boolean highlightLimits)
   {
      List<MeshDataHolder> meshes = new ArrayList<>();

      Point3D sphereCenter = new Point3D();
      Vector3D limitA = new Vector3D();
      Vector3D limitB = new Vector3D();
      Vector3D limitC = new Vector3D();

      EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1, v2, largeRadius - smallRadius, sphereCenter);

      limitA.sub(v0, sphereCenter);
      limitB.sub(v1, sphereCenter);
      limitC.sub(v2, sphereCenter);

      meshes.add(toPartialSphereMesh(sphereCenter, limitA, limitB, limitC, largeRadius, 32));

      if (highlightLimits)
         meshes.addAll(toFaceSubSphereLimits(owner, v0, v1, v2, largeRadius, smallRadius, 0.001));

      return meshes;
   }

   public static List<MeshDataHolder> toCyclicFaceSphereLimits(Face3DReadOnly face, double largeRadius, double smallRadius, double lineThickness)
   {
      List<MeshDataHolder> meshes = new ArrayList<>();

      Point3D arcCenter = new Point3D();
      Vector3D arcNormal = new Vector3D();
      Vector3D startDirection = new Vector3D();
      Vector3D endDirection = new Vector3D();
      Point3D endpoint = new Point3D();

      Vertex3DReadOnly v0 = face.getVertex(0);
      Vertex3DReadOnly v1 = face.getVertex(1);
      Vertex3DReadOnly v2 = face.getVertex(2);
      double radius = largeRadius - smallRadius;
      EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1, v2, radius, arcCenter);

      for (int edgeIndex = 0; edgeIndex < face.getNumberOfEdges(); edgeIndex++)
      {
         // Creates arcs to highlight the limits of the big spheres.
         HalfEdge3DReadOnly edge = face.getEdge(edgeIndex);
         EuclidGeometryTools.normal3DFromThreePoint3Ds(arcCenter, edge.getFirstEndpoint(), edge.getSecondEndpoint(), arcNormal);
         startDirection.sub(edge.getFirstEndpoint(), arcCenter);
         endDirection.sub(edge.getSecondEndpoint(), arcCenter);
         meshes.addAll(toSegmentedLine3DMesh(arcCenter, arcNormal, largeRadius, lineThickness, startDirection, startDirection.angle(endDirection), 32, 8));
         endpoint.scaleAdd(largeRadius / startDirection.length(), startDirection, arcCenter);
         meshes.add(MeshDataHolder.translate(MeshDataGenerator.Sphere(lineThickness, 8, 8), endpoint));
      }

      return meshes;
   }

   private static List<MeshDataHolder> toFaceSubSphereLimits(Face3DReadOnly owner, Vertex3DReadOnly v0, Vertex3DReadOnly v1, Vertex3DReadOnly v2,
                                                             double largeRadius, double smallRadius, double lineThickness)
   {
      List<MeshDataHolder> meshes = new ArrayList<>();

      Point3D arcCenter = new Point3D();
      Vector3D arcNormal = new Vector3D();
      Vector3D startDirection = new Vector3D();
      Vector3D endDirection = new Vector3D();
      Point3D endpoint = new Point3D();

      EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1, v2, largeRadius - smallRadius, arcCenter);

      Vertex3DReadOnly[] vertices = {v0, v1, v2};

      for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
      {
         // Creates arcs to highlight the limits of the big spheres.
         Vertex3DReadOnly start = vertices[vertexIndex];
         Vertex3DReadOnly end = vertices[(vertexIndex + 1) % 3];
         EuclidGeometryTools.normal3DFromThreePoint3Ds(arcCenter, start, end, arcNormal);
         startDirection.sub(start, arcCenter);
         endDirection.sub(end, arcCenter);
         meshes.addAll(toSegmentedLine3DMesh(arcCenter, arcNormal, largeRadius, lineThickness, startDirection, startDirection.angle(endDirection), 32, 8));
         endpoint.scaleAdd(largeRadius / startDirection.length(), startDirection, arcCenter);
         meshes.add(MeshDataHolder.translate(MeshDataGenerator.Sphere(lineThickness, 8, 8), endpoint));
      }

      return meshes;
   }

   public static MeshDataHolder toHalfEdgeTorus(HalfEdge3DReadOnly halfEdge, double largeRadius, double smallRadius)
   {
      Vector3D startDirection = new Vector3D();
      Vector3D endDirection = new Vector3D();
      Vector3D sphereToEdgeA = new Vector3D();
      Vector3D sphereToEdgeB = new Vector3D();

      Point3D neighborSphereCenterA = computeNeighborFaceSubSphereCenter(halfEdge, largeRadius, smallRadius);
      Point3D neighborSphereCenterB = computeNeighborFaceSubSphereCenter(halfEdge.getTwin(), largeRadius, smallRadius);

      sphereToEdgeA.sub(halfEdge.midpoint(), neighborSphereCenterA);
      sphereToEdgeB.sub(halfEdge.midpoint(), neighborSphereCenterB);
      Vector3DBasics revolutionAxis = halfEdge.getDirection(true);
      revolutionAxis.negate();
      double endRevolutionAngle = sphereToEdgeA.angle(sphereToEdgeB);

      startDirection.sub(halfEdge.getDestination(), neighborSphereCenterA);
      endDirection.sub(halfEdge.getOrigin(), neighborSphereCenterA);
      MeshDataHolder arcData = toArcPointsAndNormals(neighborSphereCenterA, largeRadius, startDirection, endDirection, 32);

      return applyRevolution(arcData, halfEdge.midpoint(), revolutionAxis, 0.0, endRevolutionAngle, 16, false);
   }

   private static Point3D computeNeighborFaceSubSphereCenter(HalfEdge3DReadOnly edge, double largeRadius, double smallRadius)
   {
      Point3D neighorSubSphereCenter = new Point3D();

      Face3DReadOnly neighbor = edge.getFace();
      int edgeIndex = neighbor.getEdges().indexOf(edge);

      Vertex3DReadOnly v0 = neighbor.getVertex(0);
      Vertex3DReadOnly v1;
      Vertex3DReadOnly v2;

      if (edgeIndex == 0)
      {
         v1 = neighbor.getVertex(1);
         v2 = neighbor.getVertex(2);
      }
      else if (edgeIndex == neighbor.getNumberOfEdges() - 1)
      {
         v1 = neighbor.getVertex(neighbor.getNumberOfEdges() - 2);
         v2 = neighbor.getVertex(neighbor.getNumberOfEdges() - 1);
      }
      else
      {
         v1 = edge.getOrigin();
         v2 = edge.getDestination();
      }

      EuclidGeometryTools.sphere3DPositionFromThreePoints(v0, v1, v2, largeRadius - smallRadius, neighorSubSphereCenter);

      return neighorSubSphereCenter;
   }

   public static List<MeshDataHolder> toVertexSphere(Vertex3DReadOnly vertex, double largeRadius, double smallRadius, boolean addVerticesMesh)
   {
      List<MeshDataHolder> meshes = new ArrayList<>();

      for (int edgeIndex = 0; edgeIndex < vertex.getNumberOfAssociatedEdges(); edgeIndex++)
      {
         meshes.add(toVertexPartialSphere(vertex, vertex.getAssociatedEdge(edgeIndex), largeRadius, smallRadius, addVerticesMesh));
      }
      return meshes;
   }

   public static MeshDataHolder toVertexPartialSphere(Vertex3DReadOnly vertex, HalfEdge3DReadOnly associatedEdge, double largeRadius, double smallRadius,
                                                      boolean addVerticesMesh)
   {
      Vector3D sphereAToEdge = new Vector3D();
      sphereAToEdge.sub(associatedEdge.midpoint(), computeNeighborFaceSubSphereCenter(associatedEdge.getTwin(), largeRadius, smallRadius));
      sphereAToEdge.normalize();
      Vector3D sphereBToEdge = new Vector3D();
      sphereBToEdge.sub(associatedEdge.midpoint(), computeNeighborFaceSubSphereCenter(associatedEdge, largeRadius, smallRadius));
      sphereBToEdge.normalize();
      double radius = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, associatedEdge.length());
      Vector3D sphereToEdge = new Vector3D();
      Point3D sphereCenter = new Point3D();
      Vector3D limitAB = new Vector3D();

      DoubleFunction<Vector3D> limitABFunction = alpha ->
      {
         sphereToEdge.interpolate(sphereAToEdge, sphereBToEdge, alpha);
         sphereToEdge.scale(radius / sphereToEdge.length());
         sphereCenter.sub(associatedEdge.midpoint(), sphereToEdge);
         limitAB.sub(vertex, sphereCenter);
         limitAB.normalize();
         return limitAB;
      };

      Vector3D limitC = new Vector3D();
      for (int faceIndex = 0; faceIndex < vertex.getNumberOfAssociatedEdges(); faceIndex++)
      {
         limitC.add(directionNeighborSubSphereToVertex(vertex, vertex.getAssociatedEdge(faceIndex).getFace(), largeRadius, smallRadius));
      }
      limitC.normalize();

      return toPartialSphereMesh(vertex, limitABFunction, limitC, smallRadius, 16, false);
   }

   private static Vector3DReadOnly directionNeighborSubSphereToVertex(Vertex3DReadOnly vertex, Face3DReadOnly neighbor, double largeRadius, double smallRadius)
   {
      int edgeIndex = neighbor.getVertices().indexOf(vertex);
      Vector3D direction = new Vector3D();
      direction.sub(vertex, computeNeighborFaceSubSphereCenter(neighbor.getEdge(edgeIndex), largeRadius, smallRadius));
      return direction;
   }

   public static List<MeshDataHolder> toSegmentedLine3DMesh(Point3DReadOnly arcCenter, Vector3DReadOnly arcNormal, double arcRadius, double thickness,
                                                            Vector3DReadOnly startDirection, double angleSpan, int resolution, int radialResolution)
   {
      SegmentedLine3DMeshDataGenerator generator = new SegmentedLine3DMeshDataGenerator(resolution, radialResolution);

      AxisAngle axisAngle = new AxisAngle(arcNormal, 0.0);
      Vector3D direction = new Vector3D();
      Point3D[] points = new Point3D[resolution];

      for (int i = 0; i < resolution; i++)
      {
         axisAngle.setAngle(angleSpan * i / (resolution - 1.0));
         axisAngle.transform(startDirection, direction);
         direction.normalize();

         Point3D point = new Point3D();
         point.scaleAdd(arcRadius, direction, arcCenter);
         points[i] = point;
      }

      generator.setLineRadius(thickness);
      generator.compute(points);
      return Arrays.asList(generator.getMeshDataHolders());
   }

   public static MeshDataHolder toPartialSphereMesh(Point3DReadOnly sphereCenter, Tuple3DReadOnly limitA, Tuple3DReadOnly limitB, Tuple3DReadOnly limitC,
                                                    double sphereRadius, int resolution)
   {
      return toPartialSphereMesh(sphereCenter, limitA, limitB, limitC, sphereRadius, resolution, false);
   }

   public static MeshDataHolder toPartialSphereMesh(Point3DReadOnly sphereCenter, Tuple3DReadOnly limitA, Tuple3DReadOnly limitB, Tuple3DReadOnly limitC,
                                                    double sphereRadius, int resolution, boolean addVerticesMesh)
   {
      return toPartialSphereMesh(sphereCenter,
                                 alpha -> interpolateVector3D(limitA, limitB, alpha),
                                 alpha -> interpolateVector3D(limitB, limitC, alpha),
                                 alpha -> interpolateVector3D(limitC, limitA, alpha),
                                 sphereRadius,
                                 resolution,
                                 addVerticesMesh);
   }

   public static MeshDataHolder toPartialSphereMesh(Point3DReadOnly sphereCenter, DoubleFunction<? extends Tuple3DReadOnly> limitABFunction,
                                                    Tuple3DReadOnly limitC, double sphereRadius, int resolution, boolean addVerticesMesh)
   {
      Vector3D limitA = new Vector3D(limitABFunction.apply(0.0));
      Vector3D limitB = new Vector3D(limitABFunction.apply(1.0));
      limitA.normalize();
      limitB.normalize();

      return toPartialSphereMesh(sphereCenter,
                                 limitABFunction,
                                 alpha -> interpolateVector3D(limitB, limitC, alpha),
                                 alpha -> interpolateVector3D(limitC, limitA, alpha),
                                 sphereRadius,
                                 resolution,
                                 addVerticesMesh);
   }

   public static MeshDataHolder toPartialSphereMesh(Point3DReadOnly sphereCenter, DoubleFunction<? extends Tuple3DReadOnly> limitABFunction,
                                                    DoubleFunction<? extends Tuple3DReadOnly> limitBCFunction,
                                                    DoubleFunction<? extends Tuple3DReadOnly> limitCAFunction, double sphereRadius, int resolution,
                                                    boolean addVerticesMesh)
   {
      List<Point3D32> points = new ArrayList<>();
      List<Vector3D32> normals = new ArrayList<>();
      List<TexCoord2f> textPoints = new ArrayList<>();

      Point3D limitAB = new Point3D();

      for (int longitude = 0; longitude < resolution - 1; longitude++)
      {
         double longitudeAlpha = longitude / (resolution - 1.0);
         int latitudeResolution = (int) Math.round(EuclidCoreTools.interpolate(resolution, 1, longitudeAlpha));

         { // latitude = 0, point is on AC limit
            Vector3D32 direction = new Vector3D32();
            direction.set(limitCAFunction.apply(1.0 - longitudeAlpha));
            direction.normalize();
            Point3D32 point = new Point3D32();
            point.scaleAdd(sphereRadius, direction, sphereCenter);
            points.add(point);
            normals.add(direction);
            textPoints.add(new TexCoord2f((float) longitudeAlpha, 0.0f));
         }

         for (int latitude = 1; latitude < latitudeResolution - 1; latitude++)
         {
            double latitudeAlpha = latitude / (latitudeResolution - 1.0);
            limitAB.set(limitABFunction.apply(latitudeAlpha));
            Vector3D32 direction = new Vector3D32();
            direction.interpolate(limitAB, limitCAFunction.apply(0.0), longitudeAlpha);
            direction.normalize();
            Point3D32 point = new Point3D32();
            point.scaleAdd(sphereRadius, direction, sphereCenter);
            points.add(point);
            normals.add(direction);
            textPoints.add(new TexCoord2f((float) longitudeAlpha, (float) latitudeAlpha));
         }

         { // latitude = latitudeResolution - 1, point is on BC limit
            Vector3D32 direction = new Vector3D32();
            direction.set(limitBCFunction.apply(longitudeAlpha));
            direction.normalize();
            Point3D32 point = new Point3D32();
            point.scaleAdd(sphereRadius, direction, sphereCenter);
            points.add(point);
            normals.add(direction);
            textPoints.add(new TexCoord2f((float) longitudeAlpha, 1.0f));
         }
      }

      { // Last vertex lies on limitC
         Vector3D32 direction = new Vector3D32();
         direction.set(limitCAFunction.apply(0.0));
         direction.normalize();
         Point3D32 point = new Point3D32();
         point.scaleAdd(sphereRadius, direction, sphereCenter);
         points.add(point);
         normals.add(direction);
         textPoints.add(new TexCoord2f(1.0f, 1.0f));
      }

      TIntArrayList triangleIndices = new TIntArrayList();

      int nextLatitudeResolution = resolution;
      int longitudeStartIndex = 0;

      for (int longitude = 0; longitude < resolution - 1; longitude++)
      {
         int latitudeResolution = nextLatitudeResolution;
         nextLatitudeResolution = (int) Math.round(EuclidCoreTools.interpolate(resolution, 1, (longitude + 1.0) / (resolution - 1.0)));
         int nextLongitudeStartIndex = longitudeStartIndex + latitudeResolution;

         for (int latitude = 0; latitude < latitudeResolution - 1; latitude++)
         {
            if (latitude < nextLatitudeResolution)
            {
               triangleIndices.add(longitudeStartIndex + latitude);
               triangleIndices.add(nextLongitudeStartIndex + latitude);
               triangleIndices.add(longitudeStartIndex + latitude + 1);
            }

            if (latitude < nextLatitudeResolution - 1)
            {
               triangleIndices.add(nextLongitudeStartIndex + latitude);
               triangleIndices.add(nextLongitudeStartIndex + latitude + 1);
               triangleIndices.add(longitudeStartIndex + latitude + 1);
            }
         }

         longitudeStartIndex += latitudeResolution;
      }

      MeshDataHolder partialSphereMesh = new MeshDataHolder(points.toArray(new Point3D32[0]),
                                                            textPoints.toArray(new TexCoord2f[0]),
                                                            triangleIndices.toArray(),
                                                            normals.toArray(new Vector3D32[0]));
      if (!addVerticesMesh)
      {
         return partialSphereMesh;
      }
      else
      {
         MeshDataBuilder meshBuilder = new MeshDataBuilder();
         meshBuilder.addMesh(partialSphereMesh);
         points.forEach(p -> meshBuilder.addTetrahedron(0.005, p));
         return meshBuilder.generateMeshDataHolder();
      }
   }

   public static MeshDataHolder toArcPointsAndNormals(Point3DReadOnly arcPosition, double arcRadius, Vector3DReadOnly startDirection,
                                                      Vector3DReadOnly endDirection, int resolution)
   {
      Point3D32[] points = new Point3D32[resolution];
      Vector3D32[] normals = new Vector3D32[resolution];

      for (int i = 0; i < resolution; i++)
      {
         double alpha = i / (resolution - 1.0);
         Vector3D32 direction = new Vector3D32();
         direction.interpolate(startDirection, endDirection, alpha);
         direction.normalize();

         Point3D32 point = new Point3D32();
         point.scaleAdd(arcRadius, direction, arcPosition);

         points[i] = point;
         normals[i] = direction;
      }

      return new MeshDataHolder(points, null, null, normals);
   }

   public static MeshDataHolder applyRevolution(MeshDataHolder subMesh, Point3DReadOnly rotationCenter, Vector3DReadOnly rotationAxis, double startAngle,
                                                double endAngle, int resolution, boolean addVerticesMesh)
   {
      int subMeshSize = subMesh.getVertices().length;
      Point3D32[] points = new Point3D32[resolution * subMeshSize];
      Vector3D32[] normals = new Vector3D32[resolution * subMeshSize];
      TexCoord2f[] textPoints = new TexCoord2f[resolution * subMeshSize];

      AxisAngle rotation = new AxisAngle();
      rotation.getAxis().set(rotationAxis);

      for (int revIndex = 0; revIndex < resolution; revIndex++)
      {
         double angle = EuclidCoreTools.interpolate(startAngle, endAngle, revIndex / (resolution - 1.0));
         rotation.setAngle(angle);

         for (int meshIndex = 0; meshIndex < subMeshSize; meshIndex++)
         {
            Point3D32 point = new Point3D32(subMesh.getVertices()[meshIndex]);
            Vector3D32 normal = new Vector3D32(subMesh.getVertexNormals()[meshIndex]);

            point.sub(rotationCenter);

            rotation.transform(point);
            rotation.transform(normal);

            point.add(rotationCenter);

            points[revIndex * subMeshSize + meshIndex] = point;
            normals[revIndex * subMeshSize + meshIndex] = normal;
            textPoints[revIndex * subMeshSize + meshIndex] = new TexCoord2f(revIndex / (resolution - 1.0f), meshIndex / (subMeshSize - 1.0f));
         }
      }

      int numberOfTriangles = 2 * resolution * subMeshSize;
      int[] triangleIndices = new int[3 * numberOfTriangles];

      int index = 0;

      for (int revIndex = 0; revIndex < resolution - 1; revIndex++)
      {
         for (int meshIndex = 0; meshIndex < subMeshSize - 1; meshIndex++)
         {
            int nextRevIndex = revIndex + 1;
            int nextMeshIndex = meshIndex + 1;

            triangleIndices[index++] = nextRevIndex * subMeshSize + meshIndex;
            triangleIndices[index++] = nextRevIndex * subMeshSize + nextMeshIndex;
            triangleIndices[index++] = revIndex * subMeshSize + nextMeshIndex;

            triangleIndices[index++] = nextRevIndex * subMeshSize + meshIndex;
            triangleIndices[index++] = revIndex * subMeshSize + nextMeshIndex;
            triangleIndices[index++] = revIndex * subMeshSize + meshIndex;
         }
      }

      MeshDataHolder partialSphereMesh = new MeshDataHolder(points, textPoints, triangleIndices, normals);

      if (!addVerticesMesh)
      {
         return partialSphereMesh;
      }
      else
      {
         MeshDataBuilder meshBuilder = new MeshDataBuilder();
         meshBuilder.addMesh(partialSphereMesh);
         Arrays.asList(points).forEach(p -> meshBuilder.addTetrahedron(0.005, p));
         return meshBuilder.generateMeshDataHolder();
      }
   }

   private static Vector3D newOrthogonalVector(Vector3DReadOnly referenceVector)
   {
      Vector3D orthogonal = new Vector3D();

      // Purposefully picking a large tolerance to ensure sanity of the cross-product.
      if (Math.abs(referenceVector.getY()) > 0.1 || Math.abs(referenceVector.getZ()) > 0.1)
         orthogonal.set(1.0, 0.0, 0.0);
      else
         orthogonal.set(0.0, 1.0, 0.0);

      orthogonal.cross(referenceVector);
      orthogonal.normalize();

      return orthogonal;
   }

   private static Vector3D interpolateVector3D(Tuple3DReadOnly tuple0, Tuple3DReadOnly tuplef, double alpha)
   {
      Vector3D interpolated = new Vector3D();
      interpolated.interpolate(tuple0, tuplef, alpha);
      return interpolated;
   }
}
