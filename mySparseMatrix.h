#ifndef MYSPARSE
#define MYSPARSE
#include "linearSystem.h"
#include "eigen-3.3.9/Eigen/Sparse"
#include "eigen-3.3.9/Eigen/Dense"
#include "eigen-3.3.9/Eigen/SVD"


class MySparseMatrix {
    std::vector< std::map<unsigned int , float> > _ASparse;

    unsigned int _rows , _columns;

public:
    MySparseMatrix() {
        _rows = _columns = 0;
    }
    MySparseMatrix( int rows , int columns ) {
        setDimensions(rows , columns);
    }
    ~MySparseMatrix() {
    }

    std::map< unsigned int , float > const & getRow( unsigned int r ) const { return _ASparse[r]; }

    void setDimensions( int rows , int columns ) {
        _rows = rows; _columns = columns;
        _ASparse.clear();
        _ASparse.resize(_rows);
    }

    float & operator() (unsigned int row , unsigned int column) {
        return _ASparse[row][column];
    }

    void convertToEigenFormat(Eigen::SparseMatrix<float> & _A) {
        // convert ad-hoc matrix to Eigen sparse format:
        {
            _A.resize(_rows , _columns);
            std::vector< Eigen::Triplet< float > > triplets;
            for( unsigned int r = 0 ; r < _rows ; ++r ) {
                for( std::map< unsigned int , float >::const_iterator it = _ASparse[r].begin() ; it != _ASparse[r].end() ; ++it ) {
                    unsigned int c = it->first;
                    float val = it->second;
                    triplets.push_back( Eigen::Triplet< float >(r,c,val) );
                }
            }
            _A.setFromTriplets( triplets.begin() , triplets.end() );
        }
    }
};




#endif
