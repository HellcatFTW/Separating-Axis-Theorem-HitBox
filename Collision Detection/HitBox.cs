using System.Drawing;
using System.Numerics;

namespace CollisionDetection;
public struct HitBox
{
    public Vector2[] Vertices { get => vertices; }
    private Vector2[] vertices = new Vector2[4];

    public float rotation;
    private Vector2 origin;

    public HitBox(Vector2 position, Rectangle rectangle, float rotation)
    {
        float height = rectangle.Height;
        float width = rectangle.Width;

        float x = position.X;
        float y = position.Y;

        Vector2 Vertex1 = new Vector2(x, y);
        Vector2 Vertex2 = new Vector2(x + width, y);
        Vector2 Vertex3 = new Vector2(x + width, y + height);
        Vector2 Vertex4 = new Vector2(x, y + height);

        this.rotation = 0;
        origin = new Vector2(width / 2, height / 2);

        vertices[0] = Vertex1 - origin;
        vertices[1] = Vertex2 - origin;
        vertices[2] = Vertex3 - origin;
        vertices[3] = Vertex4 - origin;

        ApplyRotationTransform(rotation);
    }
    public void ApplyRotationTransform(float radians)
    {
        Vector2 offset = vertices[0] + origin;

        for (int i = 0; i < vertices.Length; i++)
        {
            vertices[i] = vertices[i].RotatedBy(radians - rotation, offset);
        }
        rotation = radians;
    }
    public static bool Intersect(Vector2[] hitbox1, Vector2[] hitbox2)
    {
        return SeparatingAxisTheorem(hitbox1, hitbox2) != null;
    }
    public static Vector2? MinimumTranslationVector(Vector2[] hitbox1, Vector2[] hitbox2)
    {
        return SeparatingAxisTheorem(hitbox1, hitbox2);
    }


    #region Collision Math

    /// <summary>
    /// Takes 2 arrays of vertices for 2 convex shapes, returns a vector if the 2 shapes are intersecting. The vector is the minimum translation vector required to move <paramref name="ConvexShape2Vertices"/> so that it is no longer
    /// intersecting <paramref name="ConvexShape1Vertices"/>.
    /// </summary>
    /// <param name="ConvexShape1Vertices"></param>
    /// <param name="ConvexShape2Vertices"></param>
    /// <returns></returns>
    public static Vector2? SeparatingAxisTheorem(Vector2[] ConvexShape1Vertices, Vector2[] ConvexShape2Vertices)
    {
        Vector2[] axesToTest = new Vector2[ConvexShape1Vertices.Length + ConvexShape2Vertices.Length];

        Vector2[] normals1 = GetNormals(ConvexShape1Vertices);
        Vector2[] normals2 = GetNormals(ConvexShape2Vertices);

        for (int i = 0; i < ConvexShape1Vertices.Length; i++)
        {
            axesToTest[i] = normals1[i];
        }
        for (int i = ConvexShape1Vertices.Length; i < ConvexShape1Vertices.Length + ConvexShape2Vertices.Length; i++)
        {
            axesToTest[i] = normals2[i - ConvexShape1Vertices.Length];
        }

        float minimumProjectionDifference = float.PositiveInfinity;
        Vector2 minimumTranslationAxis = Vector2.Zero;

        foreach (Vector2 axis in axesToTest) // Which axis we are currently testing
        {
            float minimumOfFirstShape = float.PositiveInfinity;
            float maximumOfFirstShape = float.NegativeInfinity;

            float minimumOfSecondShape = float.PositiveInfinity;
            float maximumOfSecondShape = float.NegativeInfinity;

            foreach (Vector2 vertex in ConvexShape1Vertices) // get the lowest and highest result of projecting vertices of shape 1 onto the axis we are testing
            {
                float projectionLength = ProjectionLength(vertex, axis);
                maximumOfFirstShape = Math.Max(maximumOfFirstShape, projectionLength);
                minimumOfFirstShape = Math.Min(minimumOfFirstShape, projectionLength);
            }
            foreach (Vector2 vertex in ConvexShape2Vertices) // get the lowest and highest result of projecting vertices of shape 2 onto the axis we are testing
            {
                float projectionLength = ProjectionLength(vertex, axis);
                maximumOfSecondShape = Math.Max(maximumOfSecondShape, projectionLength);
                minimumOfSecondShape = Math.Min(minimumOfSecondShape, projectionLength);
            }

            if (minimumOfFirstShape < minimumOfSecondShape) // if first shape projection starts lower than second shape projection
            {
                if (maximumOfFirstShape < minimumOfSecondShape)
                {
                    return null; // 100% no collision
                }

                if ((maximumOfFirstShape - minimumOfSecondShape) < minimumProjectionDifference) // if the intersection distance on current axis is lower than the minimum, store the axis responsible and the resulting minimum.
                {
                    minimumProjectionDifference = (maximumOfFirstShape - minimumOfSecondShape);
                    minimumTranslationAxis = axis;
                }
            }
            else // if second shape projection starts lower than first shape projection
            {
                if (maximumOfSecondShape < minimumOfFirstShape)
                {
                    return null; // 100% no collision
                }

                if ((maximumOfSecondShape - minimumOfFirstShape) < minimumProjectionDifference) // if the intersection distance on current axis is lower than the minimum, store the axis responsible and the resulting minimum.
                {
                    minimumProjectionDifference = (maximumOfSecondShape - minimumOfFirstShape);
                    minimumTranslationAxis = -axis;
                }
            }
        }

        return minimumProjectionDifference * minimumTranslationAxis; // Return minimum translation vector. move object by this amount and it's not intersecting anymore.
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
            normals[i] = edges[i].RotatedBy(Math.PI/2).SafeNormalize();
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
    public static Vector2 RotatedBy(this Vector2 spinningpoint, double radians, Vector2 center = default(Vector2))
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
