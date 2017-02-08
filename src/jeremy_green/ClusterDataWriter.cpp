
#include "ClusterDataWriter.hpp"
#include "AbstractCellPopulation.hpp"
#include "VertexBasedCellPopulation.hpp"
#include "CellLabel.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
ClusterDataWriter<ELEMENT_DIM, SPACE_DIM>::ClusterDataWriter()
    : AbstractCellPopulationWriter<ELEMENT_DIM, SPACE_DIM>("clusterdata.dat")
{
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void ClusterDataWriter<ELEMENT_DIM, SPACE_DIM>::Visit(VertexBasedCellPopulation<SPACE_DIM>* pCellPopulation)
{
    // Find the centre of the tissue
    c_vector<double,2> centroid = pCellPopulation->GetCentroidOfCellPopulation();

    /*
     * To reduce any edge effects, when performing clonal analysis we will only consider cells
     * whose centres are located within some threshold distance of the centre of the tissue.
     */
    double threshold = 8.0;
    std::set<CellPtr> cells_to_consider;
    for (typename AbstractCellPopulation<SPACE_DIM>::Iterator cell_iter = pCellPopulation->Begin();
         cell_iter != pCellPopulation->End();
         ++cell_iter)
    {
        c_vector<double,2> cell_location = pCellPopulation->GetLocationOfCellCentre(*cell_iter);
        if (norm_2(cell_location - centroid) < threshold)
        {
            cells_to_consider.insert(*cell_iter);
        }
    }

    // Iterate over the cells within this region of interest
    std::set<unsigned> cells_already_considered;
    for (std::set<CellPtr>::iterator cell_iter = cells_to_consider.begin();
         cell_iter != cells_to_consider.end();
         ++cell_iter)
    {
        // If this cell has not yet been considered as part of a cluster...
        unsigned location_index = pCellPopulation->GetLocationIndexUsingCell(*cell_iter);
        if (cells_already_considered.find(location_index) == cells_already_considered.end())
        {
            // ...then check whether it has been labelled, and store its 'barcode' (ancestor index)
            cells_already_considered.insert(location_index);
            unsigned cell_label = (*cell_iter)->GetCellData()->GetItem("label");

            // If the cell has been labelled...
            if (cell_label != 0)
            {
                // ...then it is part of a labelled cluster, so we need to record data on all the cells in this cluster

                std::set<unsigned> cells_in_cluster;
                cells_in_cluster.insert(location_index);

                std::set<unsigned> neighbours_to_add = pCellPopulation->GetNeighbouringLocationIndices(*cell_iter);
                for (std::set<unsigned>::iterator iter = neighbours_to_add.begin(); iter != neighbours_to_add.end();)
                {
                   if (cells_already_considered.find(*iter) != cells_already_considered.end())
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
                           cells_already_considered.insert(*iter);
                           ++iter;
                       }
                   }
                }

                for (std::set<unsigned>::iterator iter = neighbours_to_add.begin(); iter != neighbours_to_add.end(); ++iter)
                {
                    cells_in_cluster.insert(*iter);
                }

                std::set<unsigned> neighbours_to_check = neighbours_to_add;
                std::set<unsigned>::iterator neighbour_iter = neighbours_to_check.begin();
                while (!neighbours_to_check.empty())
                {
                    CellPtr p_neighbour = pCellPopulation->GetCellUsingLocationIndex(*neighbour_iter);
                    std::set<unsigned> other_neighbours_to_add = pCellPopulation->GetNeighbouringLocationIndices(p_neighbour);
                    for (std::set<unsigned>::iterator other_iter = other_neighbours_to_add.begin(); other_iter != other_neighbours_to_add.end();)
                    {
                       if (cells_already_considered.find(*other_iter) != cells_already_considered.end())
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
                               cells_already_considered.insert(*other_iter);
                               ++other_iter;
                           }
                       }
                    }

                    for (std::set<unsigned>::iterator other_iter = other_neighbours_to_add.begin();
                         other_iter != other_neighbours_to_add.end();
                         ++other_iter)
                    {
                        cells_in_cluster.insert(*other_iter);
                        neighbours_to_check.insert(*other_iter);
                    }

                    neighbours_to_check.erase(*neighbour_iter++);
                }

                // Now we have recorded the location index of every cell in the cluster, start outputting data

                *this->mpOutStream << cell_label << "\t";

                std::map<unsigned, unsigned> ancestors_in_cluster;
                for (std::set<unsigned>::iterator iter = cells_in_cluster.begin(); iter != cells_in_cluster.end(); ++iter)
                {
                    unsigned this_ancestor = pCellPopulation->GetCellUsingLocationIndex(*iter)->GetAncestor();
                    if (ancestors_in_cluster.find(this_ancestor) == ancestors_in_cluster.end())
                    {
                        ancestors_in_cluster[this_ancestor] = 1;
                    }
                    else
                    {
                        ancestors_in_cluster[this_ancestor]++;
                    }
                }

                for (std::map<unsigned, unsigned>::iterator iter = ancestors_in_cluster.begin();
                     iter != ancestors_in_cluster.end();
                     ++iter)
                {
                    *this->mpOutStream << iter->first << "\t" << iter->second << "\t";
                }
                *this->mpOutStream << -1 << "\t";
            }
        }
    }
}

// Explicit instantiation
template class ClusterDataWriter<1,1>;
template class ClusterDataWriter<1,2>;
template class ClusterDataWriter<2,2>;
template class ClusterDataWriter<1,3>;
template class ClusterDataWriter<2,3>;
template class ClusterDataWriter<3,3>;

#include "SerializationExportWrapperForCpp.hpp"
// Declare identifier for the serializer
EXPORT_TEMPLATE_CLASS_ALL_DIMS(ClusterDataWriter)
