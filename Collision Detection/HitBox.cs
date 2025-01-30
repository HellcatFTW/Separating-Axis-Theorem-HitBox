using System.Drawing;
using System.Numerics;

namespace CollisionDetection;

public struct HitBox
{
    public Vector2[] Vertices { get => vertices; }
    private Vector2[] vertices = new Vector2[4];

    public float Rotation { get => (vertices[1] - vertices[0]).ToRotation(); }
    private Vector2 Origin { get => (vertices[2] + vertices[0]) / 2; }

    public HitBox(Vector2 position, Rectangle rectangle, float rotation)
    {
        float height = rectangle.Height;
        float width = rectangle.Width;

        float x = position.X;
        float y = position.Y;

        Vector2 Vertex0 = new Vector2(x, y);
        Vector2 Vertex1 = new Vector2(x + width, y);
        Vector2 Vertex2 = new Vector2(x + width, y + height);
        Vector2 Vertex3 = new Vector2(x, y + height);

        Vector2 origin = new Vector2(width / 2, height / 2);

        vertices[0] = Vertex0 - origin;
        vertices[1] = Vertex1 - origin;
        vertices[2] = Vertex2 - origin;
        vertices[3] = Vertex3 - origin;

        SetHitboxRotation(rotation);
    }
    public void SetHitboxRotation(float radians)
    {
        Vector2 oldOrigin = Origin;
        float difference = radians - Rotation;

        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = vertices[i].WithRotation(difference, oldOrigin);
        }
    }
    public void SetHitboxPosition(Vector2 newPosition)
    {
        float oldRotation = Rotation;
        float width = (vertices[0] - vertices[1]).Length();
        float height = (vertices[0] - vertices[^1]).Length();
        Vector2 oldOrigin = new Vector2(width / 2, height / 2);

        vertices[0] = newPosition - oldOrigin;
        vertices[1] = vertices[0] + new Vector2(width, 0);
        vertices[2] = vertices[0] + new Vector2(width, height);
        vertices[3] = vertices[0] + new Vector2(0, height);

        SetHitboxRotation(oldRotation);
    }
    public void MoveVerticesBy(Vector2 mtv)
    {
        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] += mtv;
        }
    }
    public static bool Intersect(HitBox hitbox1, HitBox hitbox2)
    {
        return SeparatingAxisTheorem(hitbox1.Vertices, hitbox2.Vertices, out _) != null;
    }
    public static Vector2? MinimumTranslationVector(HitBox hitbox1, HitBox hitbox2, out Vector2? MTVStartingPoint)
    {
        return SeparatingAxisTheorem(hitbox1.Vertices, hitbox2.Vertices, out MTVStartingPoint);
    }
    


    #region Collision Math

    /// <summary>
    /// Takes 2 arrays of vertices for 2 convex shapes, returns a vector if the 2 shapes are intersecting. The vector is the minimum translation vector required to move <paramref name="ConvexShape2Vertices"/> so that it is no longer
    /// intersecting <paramref name="ConvexShape1Vertices"/>.
    /// </summary>
    /// <param name="ConvexShape1Vertices"></param>
    /// <param name="ConvexShape2Vertices"></param>
    /// <returns></returns>
    public static Vector2? SeparatingAxisTheorem(Vector2[] ConvexShape1Vertices, Vector2[] ConvexShape2Vertices, out Vector2? MTVStartingPoint)
    {
        AxisBoolPair[] axesToTest = new AxisBoolPair[ConvexShape1Vertices.Length + ConvexShape2Vertices.Length];
        MTVStartingPoint = null;

        Vector2[] normals1 = GetNormals(ConvexShape1Vertices);
        Vector2[] normals2 = GetNormals(ConvexShape2Vertices);

        for (int i = 0; i < ConvexShape1Vertices.Length; i++)
        {
            axesToTest[i].Axis = normals1[i];
            axesToTest[i].isFromFirstShape = true;
        }
        for (int i = ConvexShape1Vertices.Length; i < ConvexShape1Vertices.Length + ConvexShape2Vertices.Length; i++)
        {
            axesToTest[i].Axis = normals2[i - ConvexShape1Vertices.Length];
            axesToTest[i].isFromFirstShape = false;
        }

        float minimumProjectionDifference = float.PositiveInfinity;
        Vector2 minimumTranslationAxis = Vector2.Zero;

        foreach (AxisBoolPair axisBoolPair in axesToTest) // Which axis we are currently testing
        {
            MinMaxPair firstShapeValues = new MinMaxPair();
            MinMaxPair secondShapeValues = new MinMaxPair();

            Vector2 axis = axisBoolPair.Axis;
            bool isCurrentAxisFromFirstShape = axisBoolPair.isFromFirstShape;

            foreach (Vector2 vertex in ConvexShape1Vertices) // get the lowest and highest result of projecting vertices of shape 1 onto the axis we are testing
            {
                float projectionLength = ProjectionLength(vertex, axis);
                firstShapeValues.UpdateValues(projectionLength, vertex);
            }
            foreach (Vector2 vertex in ConvexShape2Vertices) // get the lowest and highest result of projecting vertices of shape 2 onto the axis we are testing
            {
                float projectionLength = ProjectionLength(vertex, axis);
                secondShapeValues.UpdateValues(projectionLength, vertex);
            }

            if (firstShapeValues.minimum < secondShapeValues.minimum) // if first shape projection starts lower than second shape projection
            {
                if (firstShapeValues.maximum < secondShapeValues.minimum)
                {
                    return null; // 100% no collision
                }

                if ((firstShapeValues.maximum - secondShapeValues.minimum) < minimumProjectionDifference) // if the intersection distance on current axis is lower than the minimum, store the axis responsible and the resulting minimum.
                {
                    minimumProjectionDifference = (firstShapeValues.maximum - secondShapeValues.minimum);
                    minimumTranslationAxis = axis;

                    if (isCurrentAxisFromFirstShape)
                    {
                        MTVStartingPoint = secondShapeValues.vertexForMinimum;
                    }
                    else
                    {
                        MTVStartingPoint = firstShapeValues.vertexForMaximum;
                    }
                }
            }
            else // if second shape projection starts lower than first shape projection
            {
                if (secondShapeValues.maximum < firstShapeValues.minimum)
                {
                    return null; // 100% no collision
                }

                if ((secondShapeValues.maximum - firstShapeValues.minimum) < minimumProjectionDifference) // if the intersection distance on current axis is lower than the minimum, store the axis responsible and the resulting minimum.
                {
                    minimumProjectionDifference = (secondShapeValues.maximum - firstShapeValues.minimum);
                    minimumTranslationAxis = -axis;

                    if (isCurrentAxisFromFirstShape)
                    {
                        MTVStartingPoint = secondShapeValues.vertexForMaximum;
                    }
                    else
                    {
                        MTVStartingPoint = firstShapeValues.vertexForMinimum;
                    }
                }
            }
        }

        return minimumProjectionDifference * minimumTranslationAxis; // Return minimum translation vector. move object by this amount and it's not intersecting anymore.
    }
    private struct MinMaxPair
{
    public float minimum;
    public float maximum;

    public Vector2 vertexForMinimum;
    public Vector2 vertexForMaximum;

    public MinMaxPair()
    {
        minimum = float.PositiveInfinity;
        maximum = float.NegativeInfinity;

        vertexForMinimum = new Vector2();
        vertexForMaximum = new Vector2();
    }
    public void UpdateValues(float value, Vector2 vertex)
    {
        if (value < minimum)
        {
            minimum = value;
            vertexForMinimum = vertex;
        }
        if (value > maximum)
        {
            maximum = value;
            vertexForMaximum = vertex;
        }
    }
}
private struct AxisBoolPair
{
    public Vector2 Axis;
    public bool isFromFirstShape;

    public AxisBoolPair()
    {
        Axis = new Vector2();
        isFromFirstShape = false;
    }
    }
    /// <summary>
    /// Returns length of vector onto axis.
    /// </summary>
    /// <param name="vectorToProject"></param>
    /// <param name="axis"></param>
    /// <returns></returns>
    public static float ProjectionLength(Vector2 vectorToProject, Vector2 axis)
    {
        return Vector2.Dot(vectorToProject, axis.SafeNormalize());
    }
    public static Vector2[] GetNormals(Vector2[] vertices)
    {
        var edges = GetEdges(vertices);
        Vector2[] normals = new Vector2[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            normals[i] = edges[i].WithRotation(Math.PI / 2).SafeNormalize();
        }
        return normals;
    }
    public static Vector2[] GetEdges(Vector2[] vertices)
    {
        Vector2[] edges = new Vector2[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            edges[i] = vertices[(i + 1) % vertices.Length] - vertices[i];
        }
        return edges;
    }
}
#endregion

public static class Extensions
{
    public static float ToRotation(this Vector2 v) => (float)Math.Atan2(v.Y, v.X);
    public static Vector2 WithRotation(this Vector2 spinningpoint, double radians, Vector2 center = default(Vector2))
    {
        float cos = (float)Math.Cos(radians);
        float sin = (float)Math.Sin(radians);
        Vector2 vector = spinningpoint - center;
        Vector2 result = Vector2.Zero;
        result.X = vector.X * cos - vector.Y * sin;
        result.Y = vector.X * sin + vector.Y * cos;
        return result + center;
    }
    public static bool HasNaNs(this Vector2 vec)
    {
        if (!float.IsNaN(vec.X))
            return float.IsNaN(vec.Y);

        return true;
    }

    public static Vector2 SafeNormalize(this Vector2 v, Vector2 defaultValue = default(Vector2))
    {
        if (v == Vector2.Zero || v.HasNaNs())
            return defaultValue;

        return Vector2.Normalize(v);
    }
}