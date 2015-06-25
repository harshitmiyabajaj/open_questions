#pragma once 

#include "utils.hpp"

namespace AlgoComp {

  class InputChangeListener {
  public:
    virtual ~InputChangeListener ( ) { };

    virtual double OnInputChange ( unsigned int input_index_, double input_value_ ) = 0;
  };


  class OutputChangeListener {
  public:
    virtual ~OutputChangeListener ( ) { };

    virtual void OnOutputChange ( double _new_out_value_ ) = 0;
  };

    class CurrentInputVector {
    public:
        std::vector<double> input_vec_;
        void InitializeInputVector (const int num_predictor){
            input_vec_.clear();
            for(int i=0;i<num_predictor;i++){input_vec_.push_back(0);}
        }
        void UpdateInputVector(unsigned int input_index_, double input_value_) {
            input_vec_[input_index_]=input_value_;
        }
        const std::vector<double> & GetInputVector(){return input_vec_;}
    };

  class TertiaryRandomForest : public InputChangeListener {
  public:
    std::vector<Tree> tertiary_forest_;
    std::vector<double> forest_output_vec_;
    TertiaryRandomForest ( ) { }
    CurrentInputVector input_;
    //Will be called for initializing the Forest object
    //Can use Forest Class in utils.hpp to parse the forestfile
    void InitializeForest ( const char * const forest_filename_ ) {
        Forest forest_(forest_filename_);
        if(forest_.is_valid()){
            tertiary_forest_ = forest_.tree_vec_;
        }
        input_.InitializeInputVector(forest_.num_predictors_);
     } /// needs to be implemented
    
    //Will be called to notfiy changes in predictor values
    //Should return the updated output value
    double OnInputChange ( unsigned int input_index_, double input_value_ ) { 
        input_.UpdateInputVector(input_index_, input_value_);
        return CalcForestOutput(tertiary_forest_, input_); } /// needs to be implemented
    
    double CalcForestOutput(const std::vector<Tree> & tree_vec_,CurrentInputVector & input_){
        double prediction=0;
        forest_output_vec_.clear();
        for(int i=0;i<tree_vec_.size();i++){
            forest_output_vec_.push_back(CalcTreeOutput(tree_vec_[i],input_.GetInputVector()));
            prediction+=forest_output_vec_[i]; 
        }
        return prediction/tree_vec_.size();
    }
    
    double CalcTreeOutput(const Tree & tree_, const std::vector<double> & input_vec_){
        int node_index_=0;
        int predictor_index_;
        while(!tree_[node_index_].is_leaf_ && node_index_<tree_.size()){
            predictor_index_ = tree_[node_index_].predictor_index_;
            if(input_vec_[predictor_index_]<tree_[node_index_].boundary_value_vec_[0]){
                node_index_ = tree_[node_index_].child_node_index_vec_[0];
            }
            else if(input_vec_[predictor_index_]<=tree_[node_index_].boundary_value_vec_[1]){
                node_index_ = tree_[node_index_].child_node_index_vec_[1];
            }
            else{
                node_index_ = tree_[node_index_].child_node_index_vec_[2];
            }
        }
        return tree_[node_index_].predicted_value_;
    }
/*   
    double CalcOutputChange(const std::vector<Node>& tree_, unsigned int input_index_, double input_value_ ) {
        
        int node_index_=0;
        while (node_index_<tree_.size()) {
            //
            if(tree_[node_index_].predictor_index_==input_index_) {
                if(input_value_<tree_[node_index_].boundary_value_vec_[0]) {
                    return tree_[tree_[node_index_].child_node_index_vec_[0]].predicted_value_;
                }
                else if(input_value_<tree_[node_index_].boundary_value_vec_[1]) {
                    return tree_[tree_[node_index_].child_node_index_vec_[1]].predicted_value_;
                }
                else {
                    return tree_[tree_[node_index_].child_node_index_vec_[2]].predicted_value_;
                }
            }
            node_index_++;   
        }
        // If input change variable not found return default value ie root
        return tree_[0].predicted_value_;
    }
*/
    //_new_listener_ shoud be notified by calling _new_listener_->OnOutputChange on every change in Forest Output
    //should return true if _new_listener_ is successfully subscribed to Forest output updates
    bool SubscribeOutputChange ( OutputChangeListener * _new_listener_ ) { } ///needs to be implemented

  };

/*    class CurrentInputVector {
    public:
        std::vector<double> input_vec_;
        void InitializeInputVector (const int num_predictor){
            input_vec_.clear();
            for(int i=0;i<num_predictor;i++){input_vec_.push_back(0);}
        }
        void UpdateInputVector(unsigned int input_index_, double input_value_) {
            input_vec_[input_index_]=input_value_;
        }
        const std::vector<double> & GetInputVector(){return input_vec_;}
    };
*/
}