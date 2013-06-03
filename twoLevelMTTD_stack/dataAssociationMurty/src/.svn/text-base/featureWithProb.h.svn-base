#ifndef __FEATUREWITHPROB__
#define __FEATUREWITHPROB__
namespace BFL
{
  using namespace std;
  using namespace MatrixWrapper;

            /** FeatureWithProb
            * class containing a feature together with the probability that it is caused by any of the existing clusters
            */
    template <typename MeasVar> class FeatureWithProb
    {
            public:
                /// the value of the feature
                MeasVar feature;
                /// the probability that the feature is caused by any of the existing clusters
                double prob;
                /// Constructor 
                /*@param z feature value
                @param probValue probability of the feature
                */
                FeatureWithProb(MeasVar z, double probValue);
                /// Constructor 
                FeatureWithProb();
                bool operator <(const FeatureWithProb& b) const;

     };

////////////// IMPLEMENTATION/////////////////////////////////////
    template<typename MeasVar> 
                FeatureWithProb<MeasVar>::FeatureWithProb(MeasVar z, double probValue): 
                    feature(z), prob(probValue)
        {

        }
    template<typename MeasVar>
                FeatureWithProb<MeasVar>::FeatureWithProb() 
        {

        }
    template<typename MeasVar> bool
                FeatureWithProb<MeasVar>::operator < (const FeatureWithProb& b) const 
                { return (this->prob<b.prob); }

} // End namespace BFL

#endif // __FEATUREWITHPROB__
