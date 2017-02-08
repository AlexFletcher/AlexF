
#include "LabelledCloneSizesWriter.hpp"
#include "AbstractCellPopulation.hpp"
#include "MeshBasedCellPopulation.hpp"
#include "CaBasedCellPopulation.hpp"
#include "NodeBasedCellPopulation.hpp"
#include "PottsBasedCellPopulation.hpp"
#include "VertexBasedCellPopulation.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
LabelledCloneSizesWriter<ELEMENT_DIM, SPACE_DIM>::LabelledCloneSizesWriter()
    : AbstractCellPopulationWriter<ELEMENT_DIM, SPACE_DIM>("labelled_clone_sizes.dat")
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void LabelledCloneSizesWriter<ELEMENT_DIM, SPACE_DIM>::Visit(MeshBasedCellPopulation<ELEMENT_DIM, SPACE_DIM>* pCellPopulation)
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void LabelledCloneSizesWriter<ELEMENT_DIM, SPACE_DIM>::Visit(CaBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void LabelledCloneSizesWriter<ELEMENT_DIM, SPACE_DIM>::Visit(NodeBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void LabelledCloneSizesWriter<ELEMENT_DIM, SPACE_DIM>::Visit(PottsBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void LabelledCloneSizesWriter<ELEMENT_DIM, SPACE_DIM>::Visit(VertexBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    std::vector<int> clone_data;

    // Initialise a set of cells that have been covered
    std::set<unsigned> covered_location_indices;

    // Iterate over every cell
    for (typename AbstractCellPopulation<SPACE_DIM>::Iterator cell_iter = pCellPopulation->Begin();
         cell_iter != pCellPopulation->End();
         ++cell_iter)
    {
        // If this cell has not yet been covered...
        unsigned location_index = pCellPopulation->GetLocationIndexUsingCell(*cell_iter);
        if (covered_location_indices.find(location_index) == covered_location_indices.end())
        {
            // Start by adding it to the list of covered location indices
            covered_location_indices.insert(location_index);

            // If the cell is labelled...
            unsigned cell_label = cell_iter->GetCellData()->GetItem("label");

            std::set<unsigned> ancestors_in_this_labelled_clone;
            ancestors_in_this_labelled_clone.insert(cell_iter->GetAncestor());

            if (cell_label != 0)
            {
                // Initialise a set of location indices describing the labelled clone containing this cell
                std::set<unsigned> cells_in_labelled_clone;
                cells_in_labelled_clone.insert(location_index);

                // Generate a set of indices of neighbouring cells that share the same label and which have not yet been covered
                std::set<unsigned> neighbours_to_add = pCellPopulation->GetNeighbouringLocationIndices(*cell_iter);
                for (std::set<unsigned>::iterator iter = neighbours_to_add.begin(); iter != neighbours_to_add.end();)
                {
                   if (covered_location_indices.find(*iter) != covered_location_indices.end())
                   {
                       neighbours_to_add.erase(iter++);
                   }
                   else
                   {
                       CellPtr p_neighbour = pCellPopulation->GetCellUsingLocationIndex(*iter);
                       unsigned neighbour_label = p_neighbour->GetCellData()->GetItem("label");
                       if (neighbour_label != cell_label)
                       {
                           neighbours_to_add.erase(iter++);
                       }
                       else
                       {
                           covered_location_indices.insert(*iter);
                           ++iter;
                       }
                   }
                }

                // Every member of this set belongs to the labelled clone containing this cell
                for (std::set<unsigned>::iterator iter = neighbours_to_add.begin(); iter != neighbours_to_add.end(); ++iter)
                {
                    cells_in_labelled_clone.insert(*iter);
                    ancestors_in_this_labelled_clone.insert(pCellPopulation->GetCellUsingLocationIndex(*iter)->GetAncestor());
                }

                ///\todo improve comments
                std::set<unsigned> neighbours_to_check = neighbours_to_add;

                // Iterate over remaining neighbours to check
                std::set<unsigned>::iterator neighbour_iter = neighbours_to_check.begin();
                while (!neighbours_to_check.empty())
                {
                    // Generate a set of indices of neighbouring cells that share the same label and which have not yet been covered
                    CellPtr p_neighbour = pCellPopulation->GetCellUsingLocationIndex(*neighbour_iter);
                    std::set<unsigned> other_neighbours_to_add = pCellPopulation->GetNeighbouringLocationIndices(p_neighbour);
                    for (std::set<unsigned>::iterator other_iter = other_neighbours_to_add.begin(); other_iter != other_neighbours_to_add.end();)
                    {
                       if (covered_location_indices.find(*other_iter) != covered_location_indices.end())
                       {
                           other_neighbours_to_add.erase(other_iter++);
                       }
                       else
                       {
                           CellPtr p_other_neighbour = pCellPopulation->GetCellUsingLocationIndex(*other_iter);
                           unsigned neighbour_label = p_other_neighbour->GetCellData()->GetItem("label");
                           if (neighbour_label != cell_label)
                           {
                               other_neighbours_to_add.erase(other_iter++);
                           }
                           else
                           {
                               covered_location_indices.insert(*other_iter);
                               ++other_iter;
                           }
                       }
                    }

                    // Every member of this set belongs to the labelled clone containing this cell RENAME "this cell"
                    for (std::set<unsigned>::iterator other_iter = other_neighbours_to_add.begin(); other_iter != other_neighbours_to_add.end(); ++other_iter)
                    {
                        cells_in_labelled_clone.insert(*other_iter);
                        neighbours_to_check.insert(*other_iter);
                    }

                    // Remove this neighbour (RENAME?) from the set of neighbours whose neighbours have not yet been checked
                    neighbours_to_check.erase(*neighbour_iter++);
                }

                clone_data.push_back(cells_in_labelled_clone.size());
                for (std::set<unsigned>::iterator it = ancestors_in_this_labelled_clone.begin(); it != ancestors_in_this_labelled_clone.end(); ++it)
                {
                    clone_data.push_back(*it);
                }
                clone_data.push_back(-1);
            }
        }
    }

    for (unsigned i=0; i<clone_data.size(); i++)
    {
        *this->mpOutStream << clone_data[i] << "\t";
    }
}

// Explicit instantiation
template class LabelledCloneSizesWriter<1,1>;
template class LabelledCloneSizesWriter<1,2>;
template class LabelledCloneSizesWriter<2,2>;
template class LabelledCloneSizesWriter<1,3>;
template class LabelledCloneSizesWriter<2,3>;
template class LabelledCloneSizesWriter<3,3>;

#include "SerializationExportWrapperForCpp.hpp"
// Declare identifier for the serializer
EXPORT_TEMPLATE_CLASS_ALL_DIMS(LabelledCloneSizesWriter)
