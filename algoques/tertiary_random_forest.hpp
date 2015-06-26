#pragma once 

#include "utils.hpp"
#include <set>

namespace AlgoComp {
 class InputChangeListener; 
  class OutputChangeListener{
  public:
    double out_value_;
    virtual void OnOutputChange ( double _new_out_value_ ){out_value_ = _new_out_value_;}
    double GetOutputValue(){return out_value_;}
    virtual ~OutputChangeListener ( ){};
  };

  class InputChangeListener {
  public:
    std::vector<class OutputChangeListener *> listener_vec_;
    virtual ~InputChangeListener (){};
    virtual double OnInputChange ( unsigned int input_index_, double input_value_ )=0;
    void AddNewListener(OutputChangeListener* new_listener_){listener_vec_.push_back(new_listener_);}
    const std::vector<OutputChangeListener *> & GetListenerVector(){return listener_vec_;}
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
    std::vector< std::set<int> >forest_feature_vec_;
    double forest_delta_;
    TertiaryRandomForest ( ):forest_delta_(0) { }
    CurrentInputVector input_;
    //Will be called for initializing the Forest object
    //Can use Forest Class in utils.hpp to parse the forestfile
    void InitializeForest ( const char * const forest_filename_ ) {
        Forest forest_(forest_filename_);
        std::set<int> tree_feature_set_;
        if(forest_.is_valid()){
            tertiary_forest_ = forest_.tree_vec_;
        }
        input_.InitializeInputVector(forest_.num_predictors_);
        for(int i=0;i<tertiary_forest_.size() && !tertiary_forest_.empty();i++){
            tree_feature_set_.clear();
            for(int j=0;j<tertiary_forest_[i].size() && !tertiary_forest_[i].empty();j++){
                tree_feature_set_.insert(tertiary_forest_[i][j].predictor_index_);
            }
            forest_feature_vec_.push_back(tree_feature_set_);
        }
        forest_output_vec_.resize(tertiary_forest_.size());
     } /// needs to be implemented
    
    //Will be called to notfiy changes in predictor values
    //Should return the updated output value
    double OnInputChange ( unsigned int input_index_, double input_value_ ) { 
        forest_delta_ = 0;
        std::vector< OutputChangeListener* > listener_vec_ = this->GetListenerVector();
        input_.UpdateInputVector(input_index_, input_value_);
        forest_delta_ = CalcForestOutput(tertiary_forest_, input_, input_index_);
        for(int i=0;i<listener_vec_.size() && !listener_vec_.empty();i++){
            listener_vec_[i]->OnOutputChange(forest_delta_);
        }
        return forest_delta_; } /// needs to be implemented
    
    double CalcForestOutput(const std::vector<Tree> & tree_vec_,CurrentInputVector & input_,unsigned int input_index_){
        double prediction=0;
        for(int i=0;i<tree_vec_.size() && !tree_vec_.empty();i++){
            const bool is_in = forest_feature_vec_[i].find((int) input_index_) != forest_feature_vec_[i].end();
            if(is_in){
                forest_output_vec_[i]=CalcTreeOutput(tree_vec_[i],input_.GetInputVector());
            }
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
    
    //_new_listener_ shoud be notified by calling _new_listener_->OnOutputChange on every change in Forest Output
    //should return true if _new_listener_ is successfully subscribed to Forest output updates
    bool SubscribeOutputChange ( OutputChangeListener * _new_listener_ ) {
        _new_listener_->OnOutputChange(forest_delta_);
        return 1;
     } ///needs to be implemented

  };

}