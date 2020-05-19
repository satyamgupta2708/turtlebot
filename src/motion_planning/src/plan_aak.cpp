#include <vector>
#include <memory>
#include <algorithm>
#include <optional>
#include <cmath>
#include <unordered_map>
#include <fstream>
#include <array>
#include <iostream>

namespace turtleBot {

namespace astar {

class State;

class Action
{
public:
   Action(double cost);
   double getActionCost();
private:
   double mCost;
};

class State
{
public:

private:

};

typedef std::vector<std::shared_ptr<State>> Trajectory;

class PlanningState
{
public:
   PlanningState(std::shared_ptr<State> state);
   void setParentState(std::shared_ptr<PlanningState>& parentState);
   std::shared_ptr<PlanningState> getParentState();
   double getOptimalCostToCome();
   double setOptimalCostToCome(double value);
   std::shared_ptr<State> getState(); 
private:
   std::shared_ptr<PlanningState> mParentState;
   std::shared_ptr<State> mState;
   double mOptimalCostToCome;
};

class Heurstic
{
public:
   Heurstic(State endState);
   double getHeurstic(const std::shared_ptr<PlanningState>& state);
private:
   State mEndState;
};

class TerminationCondition
{
public:
   TerminationCondition(std::shared_ptr<PlanningState> endState);
   bool isEndState(const std::shared_ptr<PlanningState>& s);
private:
   std::shared_ptr<PlanningState> mEndState;
};

class Graph
{
public:
   Graph(std::string filename);
   std::vector<std::pair<std::shared_ptr<PlanningState>, std::shared_ptr<Action>>> getSuccessors(const std::shared_ptr<PlanningState>& state);
private:
   std::unordered_map<std::shared_ptr<PlanningState>, std::vector<std::pair<std::shared_ptr<PlanningState>, std::shared_ptr<Action>>>> mGraph;
};

class AStarSearch {
public:
   std::optional<Trajectory> search();

private:
   Trajectory tracePath(const std::shared_ptr<PlanningState>& state);

   Graph mGraph;
   std::unique_ptr<TerminationCondition> mTerminationCondition;
   std::unique_ptr<Heurstic> mHeurstic;
   std::shared_ptr<PlanningState> mStartState;
};

std::optional<Trajectory> AStarSearch::search()
{
   std::vector<std::shared_ptr<PlanningState>> openList;
   std::vector<std::shared_ptr<PlanningState>> closedList;

   openList.push_back(mStartState);

   while(!openList.empty())
   {
      auto activeStateItr = std::min(openList.begin(), openList.end(),
                                    [this](const std::shared_ptr<PlanningState>& a, const std::shared_ptr<PlanningState>& b) -> bool
                                       { 
                                          return (a->getOptimalCostToCome() + mHeurstic->getHeurstic(a)
                                                  < b->getOptimalCostToCome() + mHeurstic->getHeurstic(b));
                                       });

      auto activeState = *activeStateItr;

      if(mTerminationCondition->isEndState(activeState))
      {
         return std::make_optional(tracePath(activeState));
      }

      auto successorList = mGraph.getSuccessors(*activeStateItr);
      openList.erase(activeStateItr);

      for(auto& successor: successorList)
      {
         auto& successorState = successor.first;
         auto& successorAction = successor.second;

         double newOptimalCostToCome = activeState->getOptimalCostToCome() + successorAction->getActionCost();
         auto successorItr = std::find(openList.begin(), openList.end(), successorState);

         if(successorItr!=openList.end())
         {
            if(successorState->getOptimalCostToCome() <= newOptimalCostToCome)
               continue;
         }
         else if((successorItr = std::find(closedList.begin(), closedList.end(), successorState)) != closedList.end())
         {
            if(successorState->getOptimalCostToCome() <= newOptimalCostToCome)
               continue;

            closedList.erase(successorItr);
            openList.push_back(successorState);
         }
         else
         {
            openList.push_back(successorState);
         }

         successorState->setOptimalCostToCome(newOptimalCostToCome);
         successorState->setParentState(activeState);
      }

      closedList.push_back(activeState);
   }
   
   return std::nullopt;
}

Trajectory AStarSearch::tracePath(const std::shared_ptr<PlanningState>& state)
{
   Trajectory trajectory;
   auto currentState = state->getParentState();
   while(currentState->getParentState() != nullptr)
   {
      trajectory.push_back(state->getState());
      currentState = currentState->getParentState();
   }
   std::reverse(trajectory.begin, trajectory.end);
   return trajectory;
}

Action::Action(double cost)
: mCost(cost)
{
   //Do Nothing
}

double Action::getActionCost()
{
   return 0.0;
}

PlanningState::PlanningState(std::shared_ptr<State> state)
: mParentState(nullptr)
, mState(state)
, mOptimalCostToCome(0.0)
{
   //Do Nothing
}

void PlanningState::setParentState(std::shared_ptr<PlanningState>& parentState)
{
   mParentState = parentState;
}

std::shared_ptr<PlanningState> PlanningState::getParentState()
{
   return mParentState;
}

double PlanningState::getOptimalCostToCome()
{
   return mOptimalCostToCome;
}

double PlanningState::setOptimalCostToCome(double value)
{
   mOptimalCostToCome = value;
}

Heurstic::Heurstic(State endState)
: mEndState(endState)
{
   // Do Nothing
}

double Heurstic::getHeurstic(const std::shared_ptr<PlanningState>& state)
{
   return std::hypot(state->getState()->x() - mEndState.x(), state->getState()->y() - mEndState.y());
}

TerminationCondition::TerminationCondition(std::shared_ptr<PlanningState> endState)
: mEndState(endState)
{
   //Do Nothing
}

bool TerminationCondition::isEndState(const std::shared_ptr<PlanningState>& state)
{
   return mEndState == state;
}

Graph::Graph(std::string filename)
{
   std::ifstream bmpFile(filename);

   if(!bmpFile)
      std::cout << "Cannot open file" << std::endl;

   const size_t bmpFileHeaderSize = 54;
   std::array<char, bmpFileHeaderSize> header;
   bmpFile.read(header.data(), header.size());

   auto fileSize = *reinterpret_cast<uint32_t *>(&header[2]);
   auto dataOffset = *reinterpret_cast<uint32_t *>(&header[10]);
   auto width = *reinterpret_cast<uint32_t *>(&header[18]);
   auto height = *reinterpret_cast<uint32_t *>(&header[22]);
   auto depth = *reinterpret_cast<uint16_t *>(&header[28]);
   const std::size_t numColors = 3;
   const std::size_t redDataLocation = 2;

   std::vector<char> img(dataOffset - bmpFileHeaderSize);
   bmpFile.read(img.data(), img.size());

   auto dataSize = ((width * numColors + numColors) & (~numColors)) * height;
   img.resize(dataSize);
   bmpFile.read(img.data(), img.size());

   auto nAction = std::make_shared<Action>(1.0);
   auto sAction = std::make_shared<Action>(1.0);
   auto eAction = std::make_shared<Action>(1.0);
   auto wAction = std::make_shared<Action>(1.0);
   auto neAction = std::make_shared<Action>(std::sqrt(2));
   auto seAction = std::make_shared<Action>(std::sqrt(2));
   auto swAction = std::make_shared<Action>(std::sqrt(2));
   auto nwAction = std::make_shared<Action>(std::sqrt(2));

   std::vector<std::shared_ptr<PlanningState>> planningStateList;
   planningStateList.reserve(height*width);
   for(size_t x = 0; x < height; x++)
   {
      for(size_t y = 0; y < width; y++)
      {
         auto state = std::make_shared<State>(x, y, img[width*y + numColors*x*redDataLocation]);
         planningStateList.push_back(std::make_shared<PlanningState>(state));
      }
   }

   for(size_t x = 0; x < height; x++)
   {
      for(size_t y = 0; y < width; y++)
      {
         std::vector<std::pair<std::shared_ptr<PlanningState>, std::shared_ptr<Action>>> stateActionPair;
         if(x != 0)
         {
            stateActionPair.push_back(
               std::make_pair(planningStateList[y*width+x+1], nAction));
         }
         if(x != height-1)
         {
            stateActionPair.push_back(
               std::make_pair(planningStateList[y*width+x-1], sAction));
         }
         if(y != 0)
         {
            stateActionPair.push_back(
               std::make_pair(planningStateList[(y+1)*width+x], eAction));
         }
         if(y != width-1)
         {
            stateActionPair.push_back(
               std::make_pair(planningStateList[(y-1)*width+x], wAction));
         }
         if((x != 0) || (y=!0))
         {
            stateActionPair.push_back(
               std::make_pair(planningStateList[(y+1)*width+x+1], neAction));
         }
         if((x != height-1) || ((y=!0)))
         {
            stateActionPair.push_back(
               std::make_pair(planningStateList[(y+1)*width+x-1], seAction));
         }
         if((x != 0) || (y != width-1))
         {
            stateActionPair.push_back(
               std::make_pair(planningStateList[(y-1)*width+x+1], nwAction));
         }
         if((x != height-1) || (y != width-1))
         {
            stateActionPair.push_back(
               std::make_pair(planningStateList[(y+1)*width+x-1], swAction));
         }

         mGraph.insert({planningStateList[y*width + x], stateActionPair});         
      }
   }
   
}


std::vector<std::pair<std::shared_ptr<PlanningState>, std::shared_ptr<Action>>> 
   Graph::getSuccessors(const std::shared_ptr<PlanningState>& state)
{
   return mGraph[state];
}

} //namespace astar

} //namespace turtleBotstd::size_t length, size_t breadth