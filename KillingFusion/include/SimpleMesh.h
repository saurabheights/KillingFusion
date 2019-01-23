#ifndef SIMPLE_MESH_H
#define SIMPLE_MESH_H

#include <iostream>
#include <fstream>

#include "Eigen/Eigen"

typedef Eigen::Vector3f Vertex;

struct Triangle
{
    unsigned int idx0;
    unsigned int idx1;
    unsigned int idx2;

    Triangle(unsigned int _idx0, unsigned int _idx1, unsigned int _idx2) : idx0(_idx0), idx1(_idx1), idx2(_idx2) {}
};

class SimpleMesh
{
  public:
    SimpleMesh() {}

    SimpleMesh(std::string filepath)
    {
        // Read off file
        std::ifstream inFile(filepath);
        if (!inFile.is_open())
            std::cout << "Failed to open " << filepath;

        // write header
        std::string tmp;
        inFile >> tmp;
        int m_vertices_size, m_triangles_size;
        inFile >> m_vertices_size >> m_triangles_size >> tmp;
        m_vertices.reserve(m_vertices_size);
        m_triangles.reserve(m_triangles_size);

        // save vertices
        for (unsigned int i = 0; i < m_vertices_size; i++)
        {
            Vertex v;
            inFile >> v.x() >> v.y() >> v.z();
            m_vertices.push_back(v);
        }

        // save faces
        for (unsigned int i = 0; i < m_triangles_size; i++)
        {
            unsigned int i0, i1, i2;
            inFile >> tmp >> i0 >> i1 >> i2;
            m_triangles.push_back(Triangle(i0, i1, i2));
        }

        // close file
        inFile.close();
    }

    void Clear()
    {
        m_vertices.clear();
        m_triangles.clear();
    }

    unsigned int AddVertex(Vertex &vertex)
    {
        unsigned int vId = (unsigned int)m_vertices.size();
        m_vertices.push_back(vertex);
        return vId;
    }

    unsigned int AddFace(unsigned int idx0, unsigned int idx1, unsigned int idx2)
    {
        unsigned int fId = (unsigned int)m_triangles.size();
        Triangle triangle(idx0, idx1, idx2);
        m_triangles.push_back(triangle);
        return fId;
    }

    std::vector<Vertex> &GetVertices()
    {
        return m_vertices;
    }

    std::vector<Triangle> &GetTriangles()
    {
        return m_triangles;
    }

    bool WriteMesh(const std::string &filename)
    {
        // Write off file
        std::ofstream outFile(filename);
        if (!outFile.is_open())
            return false;

        // write header
        outFile << "OFF" << std::endl;
        outFile << m_vertices.size() << " " << m_triangles.size() << " 0" << std::endl;

        // save vertices
        for (unsigned int i = 0; i < m_vertices.size(); i++)
        {
            outFile << m_vertices[i].x() << " " << m_vertices[i].y() << " " << m_vertices[i].z() << std::endl;
        }

        // save faces
        for (unsigned int i = 0; i < m_triangles.size(); i++)
        {
            outFile << "3 " << m_triangles[i].idx0 << " " << m_triangles[i].idx1 << " " << m_triangles[i].idx2
                    << std::endl;
        }

        // close file
        outFile.close();

        return true;
    }

  private:
    std::vector<Vertex> m_vertices;
    std::vector<Triangle> m_triangles;
};

class PointCloud
{
  public:
    bool ReadFromFile(const std::string &filename)
    {
        std::ifstream is(filename, std::ios::in | std::ios::binary);
        if (!is.is_open())
        {
            std::cout << "ERROR: unable to read input file!" << std::endl;
            return false;
        }

        char nBytes;
        is.read(&nBytes, sizeof(char));

        unsigned int n;
        is.read((char *)&n, sizeof(unsigned int));

        if (nBytes == sizeof(float))
        {
            float *ps = new float[3 * n];

            is.read((char *)ps, 3 * sizeof(float) * n);

            for (unsigned int i = 0; i < n; i++)
            {
                Eigen::Vector3f p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
                m_points.push_back(p);
            }

            is.read((char *)ps, 3 * sizeof(float) * n);
            for (unsigned int i = 0; i < n; i++)
            {
                Eigen::Vector3f p(ps[3 * i + 0], ps[3 * i + 1], ps[3 * i + 2]);
                m_normals.push_back(p);
            }

            delete ps;
        }
        else
        {
            double *ps = new double[3 * n];

            is.read((char *)ps, 3 * sizeof(double) * n);

            for (unsigned int i = 0; i < n; i++)
            {
                Eigen::Vector3f p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
                m_points.push_back(p);
            }

            is.read((char *)ps, 3 * sizeof(double) * n);

            for (unsigned int i = 0; i < n; i++)
            {
                Eigen::Vector3f p((float)ps[3 * i + 0], (float)ps[3 * i + 1], (float)ps[3 * i + 2]);
                m_normals.push_back(p);
            }

            delete ps;
        }

        return true;
    }

    std::vector<Eigen::Vector3f> &GetPoints()
    {
        return m_points;
    }

    std::vector<Eigen::Vector3f> &GetNormals()
    {
        return m_normals;
    }

    unsigned int GetClosestPoint(Eigen::Vector3f &p)
    {
        unsigned int idx = 0;

        // Tip: Improve this using clustering - By Saurabh

        float min_dist = std::numeric_limits<float>::max();
        for (unsigned int i = 0; i < m_points.size(); ++i)
        {
            float dist = (p - m_points[i]).norm();
            if (min_dist > dist)
            {
                idx = i;
                min_dist = dist;
            }
        }

        return idx;
    }

  private:
    std::vector<Eigen::Vector3f> m_points;
    std::vector<Eigen::Vector3f> m_normals;
};

#endif // SIMPLE_MESH_H
