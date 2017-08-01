#ifndef MUKULPDESYSTEM_HPP_
#define MUKULPDESYSTEM_HPP_

#include "UblasCustomFunctions.hpp"
#include "ChastePoint.hpp"
#include "Node.hpp"
#include "Element.hpp"
#include <petscvec.h>

/**
 * Two coupled PDEs defining Mukul's reaction-diffusion system
 *
 * [bmp*]_t = (a_BMP - d_BMP)*[bmp*] + b_BMP*[nog*] + c_BMP + D_BMP*del^2 [bmp*],
 * [nog*]_t = a_NOG*[bmp*] + (b_NOG - d_NOG)*[nog*] + c_NOG + D_NOG*del^2 [nog*]
 */
template<unsigned DIM>
class MukulPdeSystem
{
private:

    double mABmp;
    double mBBmp;
    double mCBmp;
    double mDBmp;
    double mDiffusionCoefficientBmp;
    double mANog;
    double mBNog;
    double mCNog;
    double mDNog;
    double mDiffusionCoefficientNog;

public:

    MukulPdeSystem()
      : mABmp(1.0),
		mBBmp(1.0),
		mCBmp(1.0),
		mDBmp(1.0),
		mDiffusionCoefficientBmp(1.0),
		mANog(1.0),
		mBNog(1.0),
		mCNog(1.0),
		mDNog(1.0),
		mDiffusionCoefficientNog(1.0)
    {
    }

    ~MukulPdeSystem()
    {
    }

    double ComputeDuDtCoefficientFunction(const ChastePoint<DIM>& rX, unsigned index)
    {
        return 1.0;
    }

    double ComputeSourceTerm(const ChastePoint<DIM>& rX, c_vector<double,2>& rU, unsigned pdeIndex)
    {
        assert(pdeIndex == 0 || pdeIndex == 1);

        double source_term;
        if (pdeIndex == 0)
        {
            source_term = (mABmp - mDBmp)*rU(0) + mBBmp*rU(1) + mCBmp;
        }
        else // pdeIndex == 1
        {
            source_term = mANog*rU(0) + (mBNog - mDNog)*rU(1) + mCNog;
        }
        return source_term;
    }

    double ComputeSourceTermAtNode(const Node<DIM>& rNode, c_vector<double,2>& rU, unsigned pdeIndex)
    {
        return ComputeSourceTerm(rNode.GetPoint(), rU, pdeIndex);
    }

    c_matrix<double, DIM, DIM> ComputeDiffusionTerm(const ChastePoint<DIM>& rX, unsigned pdeIndex, Element<DIM,DIM>* pElement=NULL)
    {
        assert(pdeIndex == 0 || pdeIndex == 1);

        c_matrix<double, DIM, DIM> diffusion_term;
        if (pdeIndex == 0)
        {
            diffusion_term = mDBmp*identity_matrix<double>(DIM);
        }
        else // pdeIndex == 1
        {
            diffusion_term = mDNog*identity_matrix<double>(DIM);
        }
        return diffusion_term;
    }
};

#endif /*MUKULPDESYSTEM_HPP_*/
