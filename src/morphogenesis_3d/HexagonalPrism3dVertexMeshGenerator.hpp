
#ifndef HEXAGONALPRISM3DVERTEXMESHGENERATOR_HPP_
#define HEXAGONALPRISM3DVERTEXMESHGENERATOR_HPP_

#include <cmath>
#include <vector>
#include "MutableVertexMesh.hpp"

class HexagonalPrism3dVertexMeshGenerator
{
protected:

    /** A pointer to the mesh that this class creates. */
    MutableVertexMesh<3,3>* mpMesh;

public:

    /**
     * Constructor.
     *
     * @param numElementsInXDirection he number of rows of elements in the x direction
     * @param numElementsInYDirection the number of rows of elements in the y direction
     * @param elementSideLength the side length of each element in the xy plane
     * @param elementHeight the height of each element in the z direction
     */
    HexagonalPrism3dVertexMeshGenerator(unsigned numElementsInXDirection,
         unsigned numElementsInYDirection,
         double elementSideLength,
         double elementHeight);

    /**
     * Null constructor for derived classes to call.
     */
    HexagonalPrism3dVertexMeshGenerator()
    {
    }

    /**
     * Destructor - deletes the mesh object and pointer.
     */
    virtual ~HexagonalPrism3dVertexMeshGenerator();

    /**
     * @return a 3D mesh whose elements are hexagonal prisms
     */
    virtual MutableVertexMesh<3,3>* GetMesh();
};

#endif /*HEXAGONALPRISM3DVERTEXMESHGENERATOR_HPP_*/
