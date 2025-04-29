#include "Utils.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include "Eigen/Eigen"
#include <cmath>
#include <cassert>
using namespace std;
using namespace Eigen;
namespace PolygonalLibrary{
bool ImportMesh(PolygonalMesh& mesh)
{

    if(!ImportCell0Ds(mesh))
    {
        cout<<"Errore nella lettura del Cell0Ds";
        return false;
    }
    else
    {
        cout<<"Marker associati ai loro punti: "<<endl;
        for(auto &Marker : mesh.cell0DMarkers)
        {
            cout<<"Numero marker "<<Marker.first<<", ID associati al marker"<<endl;
            for(auto element : Marker.second)
                cout<<element<<" ";
            cout<<endl;
        }    
    }

    if(!ImportCell1Ds(mesh))
    {
        cout<<"Errore nella lettura del Cell1Ds";
        return false;
    }
    else
    {
        cout<<"Marker associati ai loro punti: "<<endl;
        for(auto &Marker : mesh.cell0DMarkers)
        {
            cout<<"Numero marker "<<Marker.first<<", ID associati al marker"<<endl;
            for(auto element : Marker.second)
                cout<<element<<" ";
            cout<<endl;
        }    
    }
    if(!ImportCell2Ds(mesh))
    {
        cout<<"Errore nella lettura del Cell2Ds";
        return false;
    }
    CheckNonZeroEdgeLengths(mesh);

    return true;

}
// ***************************************************************************
bool ImportCell0Ds(PolygonalMesh& mesh)
{
    ifstream file("Cell0Ds.csv");

    if(file.fail())
    {
        return false;
    }

    list<string> listLines;

    string line;
    while (getline(file, line))
        listLines.push_back(line);
    file.close();

    // remove header
    listLines.pop_front();

    mesh.NumCell0Ds = listLines.size();
    
    if (mesh.NumCell0Ds == 0)
    {
        cerr << "There is no cell 0D" << endl;
        return false;
    }

    mesh.Cell0DsId.reserve(mesh.NumCell0Ds);
    mesh.Cell0DsCoordinates = Eigen::MatrixXd::Zero(3, mesh.NumCell0Ds); // non ho capito

    for (string& line : listLines)
    {
        replace(line.begin(),line.end(),';',' ');
        istringstream converter(line);
        unsigned int id;
        unsigned int marker;
        Vector2d coord;

        converter >>  id >> marker >> mesh.Cell0DsCoordinates(0, id) >> mesh.Cell0DsCoordinates(1, id);

        mesh.Cell0DsId.push_back(id);
        if (marker!=0)
        {
            auto it = mesh.cell0DMarkers.find(marker);
            if(it!=mesh.cell0DMarkers.end())
            {
                mesh.cell0DMarkers[marker].push_back(id);
            }
            else
            {
                mesh.cell0DMarkers.insert({marker,{id}});
            }
        }
        return true;
    }
}
// ***************************************************************************
bool ImportCell1Ds(PolygonalMesh& mesh)
{
    ifstream file("./Cell1Ds.csv");

    if(file.fail())
    {
        return false;
    }

    list<string> listLines;
    string line;
    while (getline(file, line))
        listLines.push_back(line);

    file.close();

    // remove header
    listLines.pop_front();

    mesh.NumCell1Ds = listLines.size();

    if (mesh.NumCell1Ds == 0)
    {
        cerr << "There is no cell 1D" << endl;
        return false;
    }

    mesh.Cell1DsId.reserve(mesh.NumCell1Ds);
    mesh.Cell1DsExtrema = Eigen::MatrixXi(2, mesh.NumCell1Ds);

    for (string& line : listLines)
    {
        replace(line.begin(),line.end(),';',' ');
        istringstream converter(line);
        unsigned int id;
        unsigned int marker;
        Vector2i vertices;

        converter >>  id >> marker >>  mesh.Cell1DsExtrema(0, id) >>  mesh.Cell1DsExtrema(1, id);
        mesh.Cell1DsId.push_back(id);


        /// Memorizza i marker
        if (marker!=0)
        {
            auto it = mesh.cell1DMarkers.find(marker);
            if(it!=mesh.cell1DMarkers.end())
            {
                mesh.cell1DMarkers.end();
            }
        }
        else
        {
            mesh.cell1DMarkers.insert({marker,{id}});
        }
    }

    return true;
}
// ***************************************************************************
bool ImportCell2Ds(PolygonalMesh& mesh)
{
    ifstream file;
    file.open("./Cell2Ds.csv");

    if(file.fail())
    {
        return false;
    }
    string tmp;
    getline(file,tmp);
    unsigned int id;
    unsigned int marker;
    unsigned int num_vertices;
    unsigned int num_edges;
    while (getline(file, tmp))
    {
        replace(tmp.begin(),tmp.end(),';',' ');
        istringstream data2D(tmp);
        data2D >> id >>marker>>num_vertices;
        mesh.Cell2DsId.push_back(id);
        vector<unsigned int> Vertices(num_vertices);
        for(int i =0;i<num_vertices;i++)
            data2D>>Vertices[i];
        mesh.Cell2DsVertices.push_back(Vertices);
        data2D>>num_edges;
        vector<unsigned int> Edges(num_edges);
        for(int i=0;i<num_edges;i++)
            data2D>>Edges[i];
        mesh.Cell2DsEdges.push_back(Edges);
    }
    mesh.NumCell2Ds=mesh.Cell2DsId.size();  
    file.close();
    return true;
}
bool CheckNonZeroEdgeLengths(const PolygonalMesh& mesh)
{
    for (size_t i = 0; i < mesh.Cell1DsId.size(); ++i)
    {
        unsigned int id = mesh.Cell1DsId[i];
        unsigned int v0 = mesh.Cell1DsExtrema(0, id);
        unsigned int v1 = mesh.Cell1DsExtrema(1, id);

        Eigen::Vector2d p0(mesh.Cell0DsCoordinates(0, v0), mesh.Cell0DsCoordinates(1, v0));
        Eigen::Vector2d p1(mesh.Cell0DsCoordinates(0, v1), mesh.Cell0DsCoordinates(1, v1));

        double length = (p1 - p0).norm();

        if (length < 1e-12)
        {
            cerr << "L'edge " << id << " ha una lunghezza nulla." << endl;
            return false;
        }
    }

    return true;
}
double ComputePolygonArea(const vector<unsigned int>& vertexIds, const Eigen::MatrixXd& coordinates)
{
    double area = 0.0;
    size_t n = vertexIds.size();

    for (size_t i = 0; i < n; ++i)
    {
        const Eigen::Vector2d& p1 = coordinates.block<2,1>(0, vertexIds[i]);
        const Eigen::Vector2d& p2 = coordinates.block<2,1>(0, vertexIds[(i + 1) % n]);

        area += (p1.x() * p2.y()) - (p2.x() * p1.y());
    }

    return std::abs(area) * 0.5;
}

bool CheckNonZeroPolygonAreas(const PolygonalMesh& mesh)
{
    for (size_t i = 0; i < mesh.Cell2DsId.size(); ++i)
    {
        const auto& vertexIds = mesh.Cell2DsVertices[i];

        double area = ComputePolygonArea(vertexIds, mesh.Cell0DsCoordinates);

        if (area < 1e-12)
        {
            cerr << "Il poligono " << mesh.Cell2DsId[i] << " ha dimensione nulla." << endl;
            return false;
        }
    }

    return true;
}
}