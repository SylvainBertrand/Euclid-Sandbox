package us.ihmc.euclid.visualizers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleFunction;
import java.util.stream.Stream;

import gnu.trove.list.array.TIntArrayList;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.collision.shapeModifier.Box3DSTPBoundingVolume;
import us.ihmc.euclid.shape.collision.shapeModifier.ConvexPolytope3DSTPBoundingVolume;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.MeshDataBuilder;
import us.ihmc.graphicsDescription.MeshDataHolder;
import us.ihmc.graphicsDescription.TexCoord2f;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;

public class STPShape3DMeshBuilder
{
   public static Node toSTPBox3DMesh(Box3DSTPBoundingVolume stpBox3D)
   {
      BoxPolytope3DView boxPolytope = stpBox3D.getShape3D().asConvexPolytope();
      double largeRadius = stpBox3D.getLargeRadius();
      double smallRadius = stpBox3D.getSmallRadius();

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();

      for (int faceIndex = 0; faceIndex < boxPolytope.getNumberOfFaces(); faceIndex++)
      { // Produce faces' big sphere mesh
         meshBuilder.addMesh(toBoxFaceSphere(boxPolytope.getFace(faceIndex), largeRadius, smallRadius, true), Color.CORNFLOWERBLUE);
      }

      Set<HalfEdge3DReadOnly> processedHalfEdgeSet = new HashSet<>();

      for (int edgeIndex = 0; edgeIndex < boxPolytope.getNumberOfHalfEdges(); edgeIndex++)
      { // Building the edges' torus
         HalfEdge3DReadOnly halfEdge = boxPolytope.getHalfEdge(edgeIndex);
         if (processedHalfEdgeSet.contains(halfEdge.getTwin()))
            continue;

         processedHalfEdgeSet.add(halfEdge);
         meshBuilder.addMesh(toBoxHalfEdgeTorus(halfEdge, largeRadius, smallRadius), Color.BLUEVIOLET);
      }

      for (int vertexIndex = 0; vertexIndex < boxPolytope.getNumberOfVertices(); vertexIndex++)
      { // Building the vertices' small sphere
         Vertex3DReadOnly vertex = boxPolytope.getVertex(vertexIndex);
         meshBuilder.addMesh(toBoxVertexSphere(vertex, largeRadius, smallRadius), Color.ORANGE);
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }

   public static Node toSTPConvexPolytope3DMesh(ConvexPolytope3DSTPBoundingVolume stpConvexPolytope3D)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder();

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }

   public static MeshDataHolder toBoxFaceSphere(Face3DReadOnly face, double largeRadius, double smallRadius, boolean highlightLimits)
   {
      MeshDataBuilder meshBuilder = new MeshDataBuilder();

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

      meshBuilder.addMesh(toPartialSphereMesh(sphereCenter, limitA, limitB, limitC, largeRadius, 32));
      meshBuilder.addMesh(toPartialSphereMesh(sphereCenter, limitA, limitC, limitD, largeRadius, 32));

      if (highlightLimits)
         meshBuilder.addMesh(toBoxFaceSphereLimits(face, largeRadius, smallRadius, 0.001));

      return meshBuilder.generateMeshDataHolder();
   }

   public static MeshDataHolder toBoxFaceSphereLimits(Face3DReadOnly face, double largeRadius, double smallRadius, double lineThickness)
   {
      MeshDataBuilder meshBuilder = new MeshDataBuilder();

      Point3D arcCenter = new Point3D();
      Vector3D arcNormal = new Vector3D();
      Vector3D startDirection = new Vector3D();
      Vector3D endDirection = new Vector3D();

      double centerOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, face.getVertex(0).distance(face.getVertex(2)));
      arcCenter.scaleAdd(-centerOffset, face.getNormal(), face.getCentroid());

      for (int edgeIndex = 0; edgeIndex < face.getNumberOfEdges(); edgeIndex++)
      {
         // Creates arcs to highlight the limits of the big spheres.
         HalfEdge3DReadOnly edge = face.getEdge(edgeIndex);
         EuclidGeometryTools.normal3DFromThreePoint3Ds(arcCenter, edge.getFirstEndpoint(), edge.getSecondEndpoint(), arcNormal);
         startDirection.sub(edge.getFirstEndpoint(), arcCenter);
         endDirection.sub(edge.getSecondEndpoint(), arcCenter);
         meshBuilder.addMesh(toArcMesh(arcCenter, arcNormal, largeRadius, lineThickness, startDirection, startDirection.angle(endDirection), 32, 6));
      }

      return meshBuilder.generateMeshDataHolder();
   }

   public static MeshDataHolder toBoxHalfEdgeTorus(HalfEdge3DReadOnly halfEdge, double largeRadius, double smallRadius)
   {
      Vector3D startDirection = new Vector3D();
      Vector3D endDirection = new Vector3D();
      Vector3D sphereToEdgeA = new Vector3D();
      Vector3D sphereToEdgeB = new Vector3D();
      Point3D neighborShpereCenterA = new Point3D();
      Point3D neighborShpereCenterB = new Point3D();

      Face3DReadOnly neighborA = halfEdge.getFace();
      Face3DReadOnly neighborB = halfEdge.getTwin().getFace();

      double sphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, neighborA.getVertex(0).distance(neighborA.getVertex(2)));
      neighborShpereCenterA.scaleAdd(-sphereOffset, neighborA.getNormal(), neighborA.getCentroid());
      sphereOffset = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, neighborB.getVertex(0).distance(neighborB.getVertex(2)));
      neighborShpereCenterB.scaleAdd(-sphereOffset, neighborB.getNormal(), neighborB.getCentroid());

      sphereToEdgeA.sub(halfEdge.midpoint(), neighborShpereCenterA);
      sphereToEdgeB.sub(halfEdge.midpoint(), neighborShpereCenterB);
      Vector3DBasics revolutionAxis = halfEdge.getDirection(true);
      revolutionAxis.negate();
      double endRevolutionAngle = sphereToEdgeA.angle(sphereToEdgeB);

      startDirection.sub(halfEdge.getDestination(), neighborShpereCenterA);
      endDirection.sub(halfEdge.getOrigin(), neighborShpereCenterA);
      MeshDataHolder arcData = toArcPointsAndNormals(neighborShpereCenterA, largeRadius, startDirection, endDirection, 64);

      return applyRevolution(arcData, halfEdge.midpoint(), revolutionAxis, 0.0, endRevolutionAngle, 32, false);
   }

   public static MeshDataHolder toBoxVertexSphere(Vertex3DReadOnly vertex, double largeRadius, double smallRadius)
   {
      HalfEdge3DReadOnly edge0 = vertex.getAssociatedEdge(0);
      HalfEdge3DReadOnly edge1 = vertex.getAssociatedEdge(1);
      HalfEdge3DReadOnly edge2 = vertex.getAssociatedEdge(2);

      { // Check winding
         Vector3DBasics validationVector = edge0.getDirection(false);
         validationVector.cross(edge1.getDirection(false));
         if (validationVector.dot(edge2.getDirection(false)) < 0.0)
         {
            edge1 = vertex.getAssociatedEdge(2);
            edge2 = vertex.getAssociatedEdge(1);
         }
      }

      Face3DReadOnly neighborA = edge0.getFace();
      Face3DReadOnly neighborB = edge1.getFace();
      Face3DReadOnly neighborC = edge2.getFace();

      Point3D neighborShpereA = new Point3D();
      Point3D neighborShpereB = new Point3D();
      Point3D neighborShpereC = new Point3D();

      double sphereOffsetA = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, neighborA.getVertex(0).distance(neighborA.getVertex(2)));
      double sphereOffsetB = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, neighborB.getVertex(0).distance(neighborB.getVertex(2)));
      double sphereOffsetC = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, neighborC.getVertex(0).distance(neighborC.getVertex(2)));

      neighborShpereA.scaleAdd(-sphereOffsetA, neighborA.getNormal(), neighborA.getCentroid());
      neighborShpereB.scaleAdd(-sphereOffsetB, neighborB.getNormal(), neighborB.getCentroid());
      neighborShpereC.scaleAdd(-sphereOffsetC, neighborC.getNormal(), neighborC.getCentroid());

      HalfEdge3DReadOnly edgeAB, edgeBC, edgeCA;

      if (edge0.getTwin().getFace() == neighborB)
      {
         edgeAB = edge0;
         edgeBC = edge1;
         edgeCA = edge2;
      }
      else
      {
         edgeAB = edge1;
         edgeBC = edge2;
         edgeCA = edge0;
      }

      Vector3D sphereAToEdgeAB = new Vector3D();
      sphereAToEdgeAB.sub(edgeAB.midpoint(), neighborShpereA);
      Vector3D sphereBToEdgeAB = new Vector3D();
      sphereBToEdgeAB.sub(edgeAB.midpoint(), neighborShpereB);
      double radiusAB = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, edgeAB.length());
      Vector3D sphereABToEdge = new Vector3D();
      Point3D sphereAB = new Point3D();
      Vector3D limitAB = new Vector3D();

      DoubleFunction<Vector3D> limitABFunction = alpha ->
      {
         sphereABToEdge.interpolate(sphereAToEdgeAB, sphereBToEdgeAB, alpha);
         sphereABToEdge.scale(radiusAB / sphereABToEdge.length());
         sphereAB.sub(edgeAB.midpoint(), sphereABToEdge);
         limitAB.sub(vertex, sphereAB);
         return limitAB;
      };

      Vector3D sphereBToEdgeBC = new Vector3D();
      sphereBToEdgeBC.sub(edgeBC.midpoint(), neighborShpereB);
      Vector3D sphereCToEdgeBC = new Vector3D();
      sphereCToEdgeBC.sub(edgeBC.midpoint(), neighborShpereC);
      double radiusBC = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, edgeBC.length());
      Vector3D sphereBCToEdge = new Vector3D();
      Point3D sphereBC = new Point3D();
      Vector3D limitBC = new Vector3D();

      DoubleFunction<Vector3D> limitBCFunction = alpha ->
      {
         sphereBCToEdge.interpolate(sphereBToEdgeBC, sphereCToEdgeBC, alpha);
         sphereBCToEdge.scale(radiusBC / sphereBCToEdge.length());
         sphereBC.sub(edgeBC.midpoint(), sphereBCToEdge);
         limitBC.sub(vertex, sphereBC);
         return limitBC;
      };

      Vector3D sphereCToEdgeCA = new Vector3D();
      sphereCToEdgeCA.sub(edgeCA.midpoint(), neighborShpereC);
      Vector3D sphereAToEdgeCA = new Vector3D();
      sphereAToEdgeCA.sub(edgeCA.midpoint(), neighborShpereA);
      double radiusCA = EuclidGeometryTools.triangleIsoscelesHeight(largeRadius - smallRadius, edgeCA.length());
      Vector3D sphereCAToEdge = new Vector3D();
      Point3D sphereCA = new Point3D();
      Vector3D limitCA = new Vector3D();

      DoubleFunction<Vector3D> limitCAFunction = alpha ->
      {
         sphereCAToEdge.interpolate(sphereCToEdgeCA, sphereAToEdgeCA, alpha);
         sphereCAToEdge.scale(radiusCA / sphereCAToEdge.length());
         sphereCA.sub(edgeCA.midpoint(), sphereCAToEdge);
         limitCA.sub(vertex, sphereCA);
         return limitCA;
      };

      return toPartialSphereMesh(vertex, limitABFunction, limitBCFunction, limitCAFunction, smallRadius, 32, false);
   }

   public static MeshDataHolder toArcMesh(Point3DReadOnly arcCenter, Vector3DReadOnly arcNormal, double arcRadius, double thickness,
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
      MeshDataBuilder meshBuilder = new MeshDataBuilder();
      Stream.of(generator.getMeshDataHolders()).forEach(meshBuilder::addMesh);
      return meshBuilder.generateMeshDataHolder();
   }

   public static MeshDataHolder toPartialSphereMesh(Point3DReadOnly sphereCenter, Tuple3DReadOnly limitA, Tuple3DReadOnly limitB, Tuple3DReadOnly limitC,
                                                    double sphereRadius, int resolution)
   {
      return toPartialSphereMesh(sphereCenter, limitA, limitB, limitC, sphereRadius, resolution, false);
   }

   public static MeshDataHolder toPartialSphereMesh(Point3DReadOnly sphereCenter, Tuple3DReadOnly limitA, Tuple3DReadOnly limitB, Tuple3DReadOnly limitC,
                                                    double sphereRadius, int resolution, boolean addVerticesMesh)
   {
      List<Point3D32> points = new ArrayList<>();
      List<Vector3D32> normals = new ArrayList<>();
      List<TexCoord2f> textPoints = new ArrayList<>();

      Point3D limitAB = new Point3D();

      for (int longitude = 0; longitude < resolution; longitude++)
      {
         int latitudeResolution = (int) Math.round(EuclidCoreTools.interpolate(resolution, 1, longitude / (resolution - 1.0)));

         for (int latitude = 0; latitude < latitudeResolution; latitude++)
         {
            double abAlpha = latitudeResolution == 1 ? 0.0 : latitude / (latitudeResolution - 1.0);
            double abcAlpha = longitude / (resolution - 1.0);
            limitAB.interpolate(limitA, limitB, abAlpha);
            Vector3D32 direction = new Vector3D32();
            direction.interpolate(limitAB, limitC, abcAlpha);
            direction.normalize();
            Point3D32 point = new Point3D32();
            point.scaleAdd(sphereRadius, direction, sphereCenter);
            points.add(point);
            normals.add(direction);
            textPoints.add(new TexCoord2f(longitude / (resolution - 1), latitude == 0 ? 0 : latitude / (latitudeResolution - 1)));
         }
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
         int latitudeResolution = (int) Math.round(EuclidCoreTools.interpolate(resolution, 1, longitude / (resolution - 1.0)));

         { // latitude = 0, point is on AC limit
            Vector3D32 direction = new Vector3D32();
            direction.set(limitCAFunction.apply(1.0 - longitudeAlpha));
            direction.normalize();
            Point3D32 point = new Point3D32();
            point.scaleAdd(sphereRadius, direction, sphereCenter);
            points.add(point);
            normals.add(direction);
            textPoints.add(new TexCoord2f(longitude / (resolution - 1.0f), 0.0f));
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
            textPoints.add(new TexCoord2f(longitude / (resolution - 1.0f), latitude / (latitudeResolution - 1.0f)));
         }

         { // latitude = latitudeResolution - 1, point is on BC limit
            Vector3D32 direction = new Vector3D32();
            direction.set(limitBCFunction.apply(longitudeAlpha));
            direction.normalize();
            Point3D32 point = new Point3D32();
            point.scaleAdd(sphereRadius, direction, sphereCenter);
            points.add(point);
            normals.add(direction);
            textPoints.add(new TexCoord2f(longitude / (resolution - 1.0f), 1.0f));
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
}
