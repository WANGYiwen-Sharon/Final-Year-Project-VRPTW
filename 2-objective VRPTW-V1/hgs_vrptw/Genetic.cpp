#include <algorithm>
#include <time.h>
#include <iterator>
#include <unordered_set>

#include "Genetic.h"
#include "Params.h"
#include "Split.h"
#include "Population.h"
#include "LocalSearch.h"
#include "Individual.h"

#include <fstream>
void Genetic::UpdateReference(CostSol myCostSol)
{
    // normalize distance = (dis - min)/(max - min)
    // idealpoint[0] = nor_dis
    if((myCostSol.penalizedCost - population->normalize_penalizedcost_min) / (population->normalize_penalizedcost_max - population->normalize_penalizedcost_min) < v_IdealPoint[0]) v_IdealPoint[0] = (myCostSol.penalizedCost - population->normalize_penalizedcost_min) / (population->normalize_penalizedcost_max - population->normalize_penalizedcost_min);
    
    // normalize vehicle_num = vechile_num / max
    // idealpoint[1] = nor_veh_num
    if((myCostSol.nbRoutes - population->normalize_vehiclecost_min) / (population->normalize_vehiclecost_max - population->normalize_vehiclecost_min) < v_IdealPoint[1]) v_IdealPoint[1] = (myCostSol.nbRoutes - population->normalize_vehiclecost_min) / (population->normalize_vehiclecost_max - population->normalize_vehiclecost_min);
}

double Genetic::ScalarizingFunction(Individual * indiv1, Individual * indiv2, std::vector<double> &referencepoint)
{
    double max_fun = -1.0e+30, diff, feval, scalar_obj;
    diff = fabs((double)(indiv1->myCostSol.penalizedCost - population->normalize_penalizedcost_min) / (double)(population->normalize_penalizedcost_max - population->normalize_penalizedcost_min) - referencepoint[0]);
    
//    if(indiv2->weight_vector[0] == 0)
//        std::cout << "indiv2->weight_vector[1]: " << indiv2->weight_vector[1] <<" distance : " << indiv1->myCostSol.penalizedCost << " vehicle number : " << indiv1->myCostSol.nbRoutes << std::endl;
//        feval = 0.001 * diff;
//    else
    feval = indiv2->weight_vector[0] *  diff;
    
//    std::cout << "The first diff is " << diff << " and the feval is " << feval << std::endl;
    
    if(feval > max_fun){
        max_fun = feval;
    }

    diff = fabs((double)(indiv1->myCostSol.nbRoutes - population->normalize_vehiclecost_min) / (double)(population->normalize_vehiclecost_max - population->normalize_vehiclecost_min) - referencepoint[1]);
//    diff = fabs(indiv1->myCostSol.vehicleCost / 10 - referencepoint[1] + 0.01);
//    if(indiv2->weight_vector[1] == 0)
//        feval = 0.00001 * diff;
//    else
        feval = indiv2->weight_vector[1] *  diff;

//    std::cout << "The second diff is " << diff << " and the feval is " << feval << std::endl;
    
    if(feval > max_fun){
        max_fun = feval;
    }
    scalar_obj = max_fun;
    
//    if(indiv2->weight_vector[0] == 0) std::cout << "scalar_obj : " << scalar_obj << std::endl;
    return scalar_obj;
}


void Genetic::updateTotalpop(std::vector <Individual*> pop_tmp){
    
    for(int i = 0; i < population->totalpop.size(); i++){
        std::vector <std::vector<double>> sca_num(pop_tmp.size(), std::vector <double> (pop_tmp.size()));
        
//        if(i < 2) std::cout << "The num of subproblem is " << i << std::endl;
        
        for(int j = 0; j < (int)pop_tmp.size(); j++){
//            if(i < 2 && j < 30) std::cout << "The individual distance is " << pop_tmp[j]->myCostSol.penalizedCost  << " and the vehicle cost is " << pop_tmp[j]->myCostSol.nbRoutes << std::endl;
            
            sca_num[j][0] = ScalarizingFunction(pop_tmp[j], population->totalpop[i], v_IdealPoint);
            
//            sca_num[j][0] += params->penaltyCapacity * pop_tmp[j]->myCostSol.capacityExcess + params->penaltyDuration * pop_tmp[j]->myCostSol.durationExcess;
            sca_num[j][1] = j;
        }
        
        sort(sca_num.begin(), sca_num.end());
        
//        if(i == 0){
//            std::cout << population->totalpop[i]->weight_vector[0] << " " << population->totalpop[i]->weight_vector[1] << std::endl;
//            for(int m = 0; m < (int)pop_tmp.size(); m++){
//                std::cout << "The scaled value is " << sca_num[m][0]  << " the distance is " <<  pop_tmp[sca_num[m][1]]->myCostSol.penalizedCost << " the vehicle number is " <<  pop_tmp[sca_num[m][1]]->myCostSol.nbRoutes << std::endl;
//            }
//
//        }
         bool flag = false;
         double seed = rand() / double(RAND_MAX);
         for(int j = 0; j < pop_tmp.size(); j++){
//             int N = 999;
             if(seed < 0.8){
//                 std::cout<< "enter" << std::endl;
                if(pop_tmp[sca_num[j][1]]->isFeasible){
                    int num_repeat = 0;
                    for(int m = 0; m < i; m++){
                        if(population->totalpop[m]->myCostSol.penalizedCost == pop_tmp[sca_num[j][1]]->myCostSol.penalizedCost){
                            num_repeat++;
                        }
                    }
                    if(num_repeat < 2){
                        pop_tmp[sca_num[j][1]]->weight_vector = population->sum_weight_vector[i];
        //
                        population->totalpop[i] = pop_tmp[sca_num[j][1]];
                        
                        pop_tmp.erase(pop_tmp.begin() + sca_num[j][1]);
                        flag = true;
                        break;
                    }
                }
                
             }
             else{
                 if(!pop_tmp[sca_num[j][1]]->isFeasible){
                     int num_repeat = 0;
                     for(int m = 0; m < i; m++){
                         if(population->totalpop[m]->myCostSol.penalizedCost == pop_tmp[sca_num[j][1]]->myCostSol.penalizedCost){
                             num_repeat++;
                         }
                     }
                     if(num_repeat < 2){
                         pop_tmp[sca_num[j][1]]->weight_vector = population->sum_weight_vector[i];
         //
                         population->totalpop[i] = pop_tmp[sca_num[j][1]];

                         pop_tmp.erase(pop_tmp.begin() + sca_num[j][1]);
                         flag = true;
                         break;
                     }
                 }
                 
             }
        }
        if(seed <= 0.8){
            if(flag == false){
                for(int j = 0; j < pop_tmp.size(); j++){
                    if(pop_tmp[sca_num[j][1]]->isFeasible){
                         pop_tmp[sca_num[j][1]]->weight_vector = population->sum_weight_vector[i];
                         
                         population->totalpop[i] = pop_tmp[sca_num[j][1]];
                         pop_tmp.erase(pop_tmp.begin() + sca_num[j][1]);
                        flag = true;
                        break;
                    }
                }
            }
             if(flag == false){
                 pop_tmp[sca_num[0][1]]->weight_vector = population->sum_weight_vector[i];
                 
                 population->totalpop[i] = pop_tmp[sca_num[0][1]];
                 pop_tmp.erase(pop_tmp.begin() + sca_num[0][1]);
                flag = true;
                break;
             }
        }
        else{
            if(flag == false){
                for(int j = 0; j < pop_tmp.size(); j++){
                    if(!pop_tmp[sca_num[j][1]]->isFeasible){
                         pop_tmp[sca_num[j][1]]->weight_vector = population->sum_weight_vector[i];
                         
                         population->totalpop[i] = pop_tmp[sca_num[j][1]];
                         pop_tmp.erase(pop_tmp.begin() + sca_num[j][1]);
                        flag = true;
                        break;
                    }
                }
            }
            if(flag == false){
                pop_tmp[sca_num[0][1]]->weight_vector = population->sum_weight_vector[i];
                
                population->totalpop[i] = pop_tmp[sca_num[0][1]];
                pop_tmp.erase(pop_tmp.begin() + sca_num[0][1]);
               flag = true;
               break;
            }
        }
        
    }
    
//    population->normalize_vehiclecost_max = 0;
//    population->normalize_penalizedcost_max = 0;
//    population->normalize_penalizedcost_min = INT_MAX;
//    population->normalize_vehiclecost_min = INT_MAX;
    
    
}


void Genetic::run(int maxIterNonProd, int timeLimit)
{
//    std::string filename = "EP.csv";
//    std::ofstream myfile;
//    myfile.open(filename);
    
	if (params->nbClients == 1)
	{
		// Edge case: with 1 client, crossover will fail, genetic algorithm makes no sense
		return;
	}
    
    /* UPDATE REFERENCE */
//    std::cout<< "Start update reference." << std::endl;
    v_IdealPoint = std::vector<double>(2, 1.0e+30); // initialize ideal point
    for(int i = 0; i < population->totalpop.size(); i++){
//        std::cout<< i << std::endl;
        
        UpdateReference(population->totalpop[i]->myCostSol);
    }
    
    /* INITIAL NEIGHBORHOOD */
//    std::cout<< "Start building neighborhood." << std::endl;
    population->InitializeNeighborhood();
//    std::cout<< "Finish building neighborhood." << std::endl;
    
    /* INITIAL EXTERNAL POPULATION */
    std::vector <Individual*> EP;
    std::vector <std::vector<double>> EP_total;
    
    for(int i = 0; i < population->feasibleSubpopulation.size(); i++){
        bool if_add_offspr = true;
        int j = 0;
        for(j = 0; j < EP.size();){
            if(EP[j]->myCostSol.penalizedCost <= population->feasibleSubpopulation[i]->myCostSol.penalizedCost && EP[j]->myCostSol.nbRoutes <= population->feasibleSubpopulation[i]->myCostSol.nbRoutes){
                if_add_offspr = false;
                break;
            }
            if(EP[j]->myCostSol.penalizedCost >= population->feasibleSubpopulation[i]->myCostSol.penalizedCost && EP[j]->myCostSol.nbRoutes >= population->feasibleSubpopulation[i]->myCostSol.nbRoutes){
                EP.erase(EP.begin()+j);
                j = 0;
            }
            else{
                j++;
            }
        }
//            std::cout <<x "enter 2" << std::endl;
        if(if_add_offspr == true){
            EP.push_back(population->feasibleSubpopulation[i]);
            int j = 0;
            for(j = 0; j < EP_total.size();){
                if(EP_total[j][0] <= population->feasibleSubpopulation[i]->myCostSol.penalizedCost && EP_total[j][1] <= population->feasibleSubpopulation[i]->myCostSol.nbRoutes){
                    if_add_offspr = false;
                    break;
                }
                if(EP_total[j][0] >= population->feasibleSubpopulation[i]->myCostSol.penalizedCost && EP_total[j][1] >= population->feasibleSubpopulation[i]->myCostSol.nbRoutes){
                    EP_total.erase(EP_total.begin()+j);
                    j = 0;
                }
                else{
                    j++;
                }
            }
            if(if_add_offspr == true){
                std::vector<double> tmp;
                tmp.push_back(population->feasibleSubpopulation[i]->myCostSol.penalizedCost);
                tmp.push_back(population->feasibleSubpopulation[i]->myCostSol.nbRoutes);
                EP_total.push_back(tmp);
//                std::cout << EP_total[0][0] << " " << EP_total[0][1] << std::endl;
            }
        }
    }
    
	// Do iterations of the Genetic Algorithm, until more then maxIterNonProd consecutive iterations without improvement or a time limit (in seconds) is reached
	int nbIterNonProd = 1;
    int nbIter = 0;
    for (nbIter = 0; !params->isTimeLimitExceeded(); nbIter++)
	{
//        if(nbIter % 10 == 0){
////            std::cout << "starting iteration " << std::endl;
//            std::cout << "The reference point is " << v_IdealPoint[0] << " " << v_IdealPoint[1] << std::endl;
//            std::cout << "The size of population is " << population->totalpop.size() << std::endl;
//            for(int i = 0; i < population->totalpop.size(); i++){
////                std::cout << "weight vector 1 : " << population->totalpop[i]->weight_vector[0] << " weight vector 2: " << population->totalpop[i]->weight_vector[1] << std::endl;
//                std::cout << "penalizedcost: " << population->totalpop[i]->myCostSol.penalizedCost << " vehiclecost: " << population->totalpop[i]->myCostSol.nbRoutes << std::endl;
//            }
//        }
        std::vector <Individual*> pop_tmp;
        int restart_num = 0;
        for(int i = 0; i < population->totalpop.size(); i++){
            if(!params->isTimeLimitExceeded()){
                Individual * subproblem;
                subproblem = population->totalpop[i];
                
                Individual * parent1;
                Individual * parent2;
                int N = 999;
                double rtn1 = rand() % (N + 1) / (float)(N + 1);
                double rtn2 = rand() % (N + 1) / (float)(N + 1);
                if(rtn1 < 0.8) parent1 = population->SelectMatingPool(*subproblem);
                else{
                    int place1 = std::rand() % (population->totalpop.size());
                    parent1 = population->totalpop[place1];
                }
                if(rtn2 < 0.8) parent2 = population->SelectMatingPool(*subproblem);
                else{
                    int place2 = std::rand() % (population->totalpop.size());
                    parent2 = population->totalpop[place2];
                }
//                std::cout << "finishing choosing parents " << std::endl;
                /* SELECTION AND CROSSOVER */
                // First select parents using getNonIdenticalParentsBinaryTournament
                // Then use the selected parents to create new individuals using OX and SREX
                // Finally select the best new individual based on bestOfSREXAndOXCrossovers
                
                Individual* offspring = bestOfSREXAndOXCrossovers(std::make_pair(parent1, parent2));
//                std::cout << "finishing creating offspring " << std::endl;
                /* LOCAL SEARCH */
                // Run the Local Search on the new individual
                localSearch->run(offspring, params->penaltyCapacity, params->penaltyTimeWarp);
//                std::cout << "finishing local search" << std::endl;
                
                
                /* UPDATE MAX AND MIN COST*/
                if(offspring->myCostSol.penalizedCost < population->normalize_penalizedcost_min) population->normalize_penalizedcost_min = offspring->myCostSol.penalizedCost;
                if(offspring->myCostSol.nbRoutes < population->normalize_vehiclecost_min) population->normalize_vehiclecost_min = offspring->myCostSol.nbRoutes;
                if(offspring->myCostSol.penalizedCost > population->normalize_penalizedcost_max) population->normalize_penalizedcost_max = offspring->myCostSol.penalizedCost;
                if(offspring->myCostSol.nbRoutes > population->normalize_vehiclecost_max) population->normalize_vehiclecost_max = offspring->myCostSol.nbRoutes;
                
                /* UPDATE REFERENCE */
//                std::cout << "----- STARTING UPDATING REFERENCES" << std::endl;
                UpdateReference(offspring->myCostSol);
                
//                std::cout << "finishing updating reference" << std::endl;
                /* INSERT INTO TEMPORARY POPULATION */
//                std::cout << "offspring penalized cost is " << offspring->myCostSol.penalizedCost << std::endl;
                Individual * myoffspring = new Individual;
                *myoffspring= *offspring;
//                std::cout << "myoffspring penalized cost is " << myoffspring->myCostSol.penalizedCost << std::endl;
                pop_tmp.push_back(myoffspring);
//                std::cout << "finishing updating temperary population" << std::endl;
                
                /* UPDATE EP */
                if(myoffspring->isFeasible){
                    bool if_add_offspr = true;
                    int j = 0;
                    for(j = 0; j < EP.size();){
                        if(EP[j]->myCostSol.penalizedCost <= myoffspring->myCostSol.penalizedCost && EP[j]->myCostSol.nbRoutes <= myoffspring->myCostSol.nbRoutes){
                            if_add_offspr = false;
                            break;
                        }
                        if(EP[j]->myCostSol.penalizedCost >= myoffspring->myCostSol.penalizedCost && EP[j]->myCostSol.nbRoutes >= myoffspring->myCostSol.nbRoutes){
                            EP.erase(EP.begin()+j);
                            j = 0;
                        }
                        else{
                            j++;
                        }
                    }
            //            std::cout << "enter 2" << std::endl;
                    if(if_add_offspr == true){
                        restart_num++;
                        EP.push_back(myoffspring);
                        int j = 0;
                        for(j = 0; j < EP_total.size();){
                            if(EP_total[j][0] <= myoffspring->myCostSol.penalizedCost && EP_total[j][1] <= myoffspring->myCostSol.nbRoutes){
                                if_add_offspr = false;
                                break;
                            }
                            if(EP_total[j][0] >= myoffspring->myCostSol.penalizedCost && EP_total[j][1] >= myoffspring->myCostSol.nbRoutes){
                                EP_total.erase(EP_total.begin()+j);
                                j = 0;
                            }
                            else{
                                j++;
                            }
                        }
                        if(if_add_offspr == true){
                            std::vector<double> tmp;
                            tmp.push_back(myoffspring->myCostSol.penalizedCost);
                            tmp.push_back(myoffspring->myCostSol.nbRoutes);
                            EP_total.push_back(tmp);
            //                std::cout << EP_total[0][0] << " " << EP_total[0][1] << std::endl;
                        }
                    }
                }
            }
        }
        
        /* SELECTION OPERATOR */
        for(int i = 0; i < population->totalpop.size(); i++){
            pop_tmp.push_back(population->totalpop[i]);
        }
        updateTotalpop(pop_tmp);
        
        /* UPDATE NEIGHBORHOOD */
        population->InitializeNeighborhood();
        
        /* DIVERSIFICATION, PENALTY MANAGEMENT AND TRACES */
        // Update the penaltyTimeWarp and penaltyCapacity every 100 iterations
        if (nbIter % 30 == 0)
        {
            population->managePenalties();
        }
        if (nbIter % 50 == 0)
        {
            
            std::cout << "nbIter: " << nbIter << std::endl;
            for(int i = 0; i < EP_total.size(); i++){
                //    population->normalize_vehiclecost_max = 0;
                //    population->normalize_penalizedcost_max = 0;
                //    population->normalize_penalizedcost_min = INT_MAX;
                //    population->normalize_vehiclecost_min = INT_MAX;
                std::cout << "penalizedcost: " << EP_total[i][0] << " "<<(EP_total[i][0] - population->normalize_penalizedcost_min) / (population->normalize_penalizedcost_max - population->normalize_penalizedcost_min) << " vehiclecost: " << EP_total[i][1] << " " << (EP_total[i][1] - population->normalize_vehiclecost_min) / (population->normalize_vehiclecost_max - population->normalize_vehiclecost_min) << std::endl;
            }
            if(nbIter == 150){
                std::cout << std::endl;
                for(int i = 0; i < population->totalpop.size(); i++){
                    std::cout << "penalizedcost: " << population->totalpop[i]->myCostSol.penalizedCost << " vehiclecost: " << population->totalpop[i]->myCostSol.nbRoutes << std::endl;
                }
            }
            

        }
        
        /* RESTART THE POPULATION */
//        if(restart_num == 0){
//            nbIterNonProd++;
//            if(nbIterNonProd == 500){
//                std::cout << "----- STARTING RESTART" << std::endl;
//                population->restart();
//                nbIterNonProd = 1;
//                EP.clear();
//                for(int i = 0; i < population->feasibleSubpopulation.size(); i++){
//                    bool if_add_offspr = true;
//                    int j = 0;
//                    for(j = 0; j < EP.size();){
//                        if(EP[j]->myCostSol.penalizedCost <= population->feasibleSubpopulation[i]->myCostSol.penalizedCost && EP[j]->myCostSol.nbRoutes <= population->feasibleSubpopulation[i]->myCostSol.nbRoutes){
//                            if_add_offspr = false;
//                            break;
//                        }
//                        if(EP[j]->myCostSol.penalizedCost >= population->feasibleSubpopulation[i]->myCostSol.penalizedCost && EP[j]->myCostSol.nbRoutes >= population->feasibleSubpopulation[i]->myCostSol.nbRoutes){
//                            EP.erase(EP.begin()+j);
//                            j = 0;
//                        }
//                        else{
//                            j++;
//                        }
//                    }
//            //            std::cout <<x "enter 2" << std::endl;
//                    if(if_add_offspr == true){
//                        EP.push_back(population->feasibleSubpopulation[i]);
//                        int j = 0;
//                        for(j = 0; j < EP_total.size();){
//                            if(EP_total[j][0] <= population->feasibleSubpopulation[i]->myCostSol.penalizedCost && EP_total[j][1] <= population->feasibleSubpopulation[i]->myCostSol.nbRoutes){
//                                if_add_offspr = false;
//                                break;
//                            }
//                            if(EP_total[j][0] >= population->feasibleSubpopulation[i]->myCostSol.penalizedCost && EP_total[j][1] >= population->feasibleSubpopulation[i]->myCostSol.nbRoutes){
//                                EP_total.erase(EP_total.begin()+j);
//                                j = 0;
//                            }
//                            else{
//                                j++;
//                            }
//                        }
//                        if(if_add_offspr == true){
//                            std::vector<double> tmp;
//                            tmp.push_back(population->feasibleSubpopulation[i]->myCostSol.penalizedCost);
//                            tmp.push_back(population->feasibleSubpopulation[i]->myCostSol.nbRoutes);
//                            EP_total.push_back(tmp);
//            //                std::cout << EP_total[0][0] << " " << EP_total[0][1] << std::endl;
//                        }
//                    }
//                }
//            }
//        }

		
		// Print the state of the population every 500 iterations
//		if (nbIter % 500 == 0)
//		{
//			population->printState(nbIter, nbIterNonProd);
//		}
//        if (nbIter % 1000 == 0){
//            myfile << params->getTimeElapsedSeconds() << ";";
//            if (population->getBestFeasible() != nullptr)
//            {
//                myfile << population->getBestFeasible()->myCostSol.penalizedCost << std::endl;
//            }
//        }
		// Log the current population to a .csv file every logPoolInterval iterations (if logPoolInterval is not 0)
//		if (params->config.logPoolInterval > 0 && nbIter % params->config.logPoolInterval == 0)
//		{
//			population->exportPopulation(nbIter, params->config.pathSolution + ".log.csv");
//		}

		
		/* OTHER PARAMETER CHANGES*/
		// Increase the nbGranular by growNbGranularSize (and set the correlated vertices again) every certain number of iterations, if growNbGranularSize is greater than 0
		if (nbIter > 0 && params->config.growNbGranularSize != 0 && (
			(params->config.growNbGranularAfterIterations > 0 && nbIter % params->config.growNbGranularAfterIterations == 0) ||
			(params->config.growNbGranularAfterNonImprovementIterations > 0 && nbIterNonProd % params->config.growNbGranularAfterNonImprovementIterations == 0)
			))
		{
			// Note: changing nbGranular also changes how often the order is reshuffled
			params->config.nbGranular += params->config.growNbGranularSize;
			params->SetCorrelatedVertices();
		}

		// Increase the minimumPopulationSize by growPopulationSize every certain number of iterations, if growPopulationSize is greater than 0
		if (nbIter > 0 && params->config.growPopulationSize != 0 && (
			(params->config.growPopulationAfterIterations > 0 && nbIter % params->config.growPopulationAfterIterations == 0) || 
			(params->config.growPopulationAfterNonImprovementIterations > 0 && nbIterNonProd % params->config.growPopulationAfterNonImprovementIterations == 0)
			))
		{
			// This will automatically adjust after some iterations
			params->config.minimumPopulationSize += params->config.growPopulationSize;
		}
	}
    
    for(int i = 0; i < EP_total.size(); i++){
        std::cout << EP_total[i][0] << " " << EP_total[i][1] << std::endl;
    }
    
//    std::cout << "----- GENETIC ALGORITHM FINISHED AFTER " << nbIter << " ITERATIONS. TIME SPENT: " << (double)(clock() - params->startCPUTime) / (double)CLOCKS_PER_SEC << std::endl;
}

Individual* Genetic::crossoverOX(std::pair<const Individual*, const Individual*> parents)
{
	// Picking the start and end of the crossover zone
	int start = params->rng() % params->nbClients;
	int end = params->rng() % params->nbClients;

	// If the start and end overlap, change the end of the crossover zone
	while (end == start)
	{
		end = params->rng() % params->nbClients;
	}

	// Create two individuals using OX
	doOXcrossover(candidateOffsprings[2], parents, start, end);
	doOXcrossover(candidateOffsprings[3], parents, start, end);

	// Return the best individual of the two, based on penalizedCost
	return candidateOffsprings[2]->myCostSol.penalizedCost < candidateOffsprings[3]->myCostSol.penalizedCost
		? candidateOffsprings[2]
		: candidateOffsprings[3];
}

void Genetic::doOXcrossover(Individual* result, std::pair<const Individual*, const Individual*> parents, int start, int end)
{
	// Frequency vector to track the clients which have been inserted already
	std::vector<bool> freqClient = std::vector<bool>(params->nbClients + 1, false);

	// Copy in place the elements from start to end (possibly "wrapping around" the end of the array)
	int j = start;
	while (j % params->nbClients != (end + 1) % params->nbClients)
	{
		result->chromT[j % params->nbClients] = parents.first->chromT[j % params->nbClients];
		// Mark the client as copied
		freqClient[result->chromT[j % params->nbClients]] = true;
		j++;
	}

	// Fill the remaining elements in the order given by the second parent
	for (int i = 1; i <= params->nbClients; i++)
	{
		// Check if the next client is already copied in place
		int temp = parents.second->chromT[(end + i) % params->nbClients];
		// If the client is not yet copied in place, copy in place now
		if (freqClient[temp] == false)
		{
			result->chromT[j % params->nbClients] = temp;
			j++;
		}
	}

	// Completing the individual with the Split algorithm
	split->generalSplit(result, params->nbVehicles);
}

Individual* Genetic::crossoverSREX(std::pair<const Individual*, const Individual*> parents)
{
	// Get the number of routes of both parents
	int nOfRoutesA = parents.first->myCostSol.nbRoutes;
	int nOfRoutesB = parents.second->myCostSol.nbRoutes;

	// Picking the start index of routes to replace of parent A
	// We like to replace routes with a large overlap of tasks, so we choose adjacent routes (they are sorted on polar angle)
	int startA = params->rng() % nOfRoutesA;
	int nOfMovedRoutes = std::min(nOfRoutesA, nOfRoutesB) == 1 ? 1 : params->rng() % (std::min(nOfRoutesA - 1, nOfRoutesB - 1)) + 1; // Prevent not moving any routes
	int startB = startA < nOfRoutesB ? startA : 0;

	std::unordered_set<int> clientsInSelectedA;
	for (int r = 0; r < nOfMovedRoutes; r++)
	{
		// Insert the first 
		clientsInSelectedA.insert(parents.first->chromR[(startA + r) % nOfRoutesA].begin(),
			parents.first->chromR[(startA + r) % nOfRoutesA].end());
	}

	std::unordered_set<int> clientsInSelectedB;
	for (int r = 0; r < nOfMovedRoutes; r++)
	{
		clientsInSelectedB.insert(parents.second->chromR[(startB + r) % nOfRoutesB].begin(),
			parents.second->chromR[(startB + r) % nOfRoutesB].end());
	}

	bool improved = true;
	while (improved)
	{
		// Difference for moving 'left' in parent A
		const int differenceALeft = static_cast<int>(std::count_if(parents.first->chromR[(startA - 1 + nOfRoutesA) % nOfRoutesA].begin(),
			parents.first->chromR[(startA - 1 + nOfRoutesA) % nOfRoutesA].end(),
			[&clientsInSelectedB](int c) { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); }))
			- static_cast<int>(std::count_if(parents.first->chromR[(startA + nOfMovedRoutes - 1) % nOfRoutesA].begin(),
				parents.first->chromR[(startA + nOfMovedRoutes - 1) % nOfRoutesA].end(),
				[&clientsInSelectedB](int c) { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); }));

		// Difference for moving 'right' in parent A
		const int differenceARight = static_cast<int>(std::count_if(parents.first->chromR[(startA + nOfMovedRoutes) % nOfRoutesA].begin(),
			parents.first->chromR[(startA + nOfMovedRoutes) % nOfRoutesA].end(),
			[&clientsInSelectedB](int c) { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); }))
			- static_cast<int>(std::count_if(parents.first->chromR[startA].begin(),
				parents.first->chromR[startA].end(),
				[&clientsInSelectedB](int c) { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); }));

		// Difference for moving 'left' in parent B
		const int differenceBLeft = static_cast<int>(std::count_if(parents.second->chromR[(startB - 1 + nOfMovedRoutes) % nOfRoutesB].begin(),
			parents.second->chromR[(startB - 1 + nOfMovedRoutes) % nOfRoutesB].end(),
			[&clientsInSelectedA](int c) { return clientsInSelectedA.find(c) != clientsInSelectedA.end(); }))
			- static_cast<int>(std::count_if(parents.second->chromR[(startB - 1 + nOfRoutesB) % nOfRoutesB].begin(),
				parents.second->chromR[(startB - 1 + nOfRoutesB) % nOfRoutesB].end(),
				[&clientsInSelectedA](int c) { return clientsInSelectedA.find(c) != clientsInSelectedA.end(); }));

		// Difference for moving 'right' in parent B
		const int differenceBRight = static_cast<int>(std::count_if(parents.second->chromR[startB].begin(),
			parents.second->chromR[startB].end(),
			[&clientsInSelectedA](int c) { return clientsInSelectedA.find(c) != clientsInSelectedA.end(); }))
			- static_cast<int>(std::count_if(parents.second->chromR[(startB + nOfMovedRoutes) % nOfRoutesB].begin(),
				parents.second->chromR[(startB + nOfMovedRoutes) % nOfRoutesB].end(),
				[&clientsInSelectedA](int c) { return clientsInSelectedA.find(c) != clientsInSelectedA.end(); }));

		const int bestDifference = std::min({ differenceALeft, differenceARight, differenceBLeft, differenceBRight });

		if (bestDifference < 0)
		{
			if (bestDifference == differenceALeft)
			{
				for (int c : parents.first->chromR[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
				{
					clientsInSelectedA.erase(clientsInSelectedA.find(c));
				}
				startA = (startA - 1 + nOfRoutesA) % nOfRoutesA;
				for (int c : parents.first->chromR[startA])
				{
					clientsInSelectedA.insert(c);
				}
			}
			else if (bestDifference == differenceARight)
			{
				for (int c : parents.first->chromR[startA])
				{
					clientsInSelectedA.erase(clientsInSelectedA.find(c));
				}
				startA = (startA + 1) % nOfRoutesA;
				for (int c : parents.first->chromR[(startA + nOfMovedRoutes - 1) % nOfRoutesA])
				{
					clientsInSelectedA.insert(c);
				}
			}
			else if (bestDifference == differenceBLeft)
			{
				for (int c : parents.second->chromR[(startB + nOfMovedRoutes - 1) % nOfRoutesB])
				{
					clientsInSelectedB.erase(clientsInSelectedB.find(c));
				}
				startB = (startB - 1 + nOfRoutesB) % nOfRoutesB;
				for (int c : parents.second->chromR[startB])
				{
					clientsInSelectedB.insert(c);
				}
			}
			else if (bestDifference == differenceBRight)
			{
				for (int c : parents.second->chromR[startB])
				{
					clientsInSelectedB.erase(clientsInSelectedB.find(c));
				}
				startB = (startB + 1) % nOfRoutesB;
				for (int c : parents.second->chromR[(startB + nOfMovedRoutes - 1) % nOfRoutesB])
				{
					clientsInSelectedB.insert(c);
				}
			}
		}
		else
		{
			improved = false;
		}
	}

	// Identify differences between route sets
	std::unordered_set<int> clientsInSelectedANotB;
	std::copy_if(clientsInSelectedA.begin(), clientsInSelectedA.end(),
		std::inserter(clientsInSelectedANotB, clientsInSelectedANotB.end()),
		[&clientsInSelectedB](int c) { return clientsInSelectedB.find(c) == clientsInSelectedB.end(); });

	std::unordered_set<int> clientsInSelectedBNotA;
	std::copy_if(clientsInSelectedB.begin(), clientsInSelectedB.end(),
		std::inserter(clientsInSelectedBNotA, clientsInSelectedBNotA.end()),
		[&clientsInSelectedA](int c) { return clientsInSelectedA.find(c) == clientsInSelectedA.end(); });

	// Replace selected routes from parent A with routes from parent B
	for (int r = 0; r < nOfMovedRoutes; r++)
	{
		int indexA = (startA + r) % nOfRoutesA;
		int indexB = (startB + r) % nOfRoutesB;
		candidateOffsprings[0]->chromR[indexA].clear();
		candidateOffsprings[1]->chromR[indexA].clear();

		for (int c : parents.second->chromR[indexB])
		{
			candidateOffsprings[0]->chromR[indexA].push_back(c);
			if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
			{
				candidateOffsprings[1]->chromR[indexA].push_back(c);
			}
		}
	}

	// Move routes from parent A that are kept
	for (int r = nOfMovedRoutes; r < nOfRoutesA; r++)
	{
		int indexA = (startA + r) % nOfRoutesA;
		candidateOffsprings[0]->chromR[indexA].clear();
		candidateOffsprings[1]->chromR[indexA].clear();

		for (int c : parents.first->chromR[indexA])
		{
			if (clientsInSelectedBNotA.find(c) == clientsInSelectedBNotA.end())
			{
				candidateOffsprings[0]->chromR[indexA].push_back(c);
			}
			candidateOffsprings[1]->chromR[indexA].push_back(c);
		}
	}

	// Delete any remaining routes that still lived in offspring
	for (int r = nOfRoutesA; r < params->nbVehicles; r++)
	{
		candidateOffsprings[0]->chromR[r].clear();
		candidateOffsprings[1]->chromR[r].clear();
	}

	// Step 3: Insert unplanned clients (those that were in the removed routes of A but not the inserted routes of B)
	insertUnplannedTasks(candidateOffsprings[0], clientsInSelectedANotB);
	insertUnplannedTasks(candidateOffsprings[1], clientsInSelectedANotB);

	candidateOffsprings[0]->evaluateCompleteCost();
	candidateOffsprings[1]->evaluateCompleteCost();

	return candidateOffsprings[0]->myCostSol.penalizedCost < candidateOffsprings[1]->myCostSol.penalizedCost
		? candidateOffsprings[0]
		: candidateOffsprings[1];
}

void Genetic::insertUnplannedTasks(Individual* offspring, std::unordered_set<int> unplannedTasks)
{
	// Initialize some variables
	int newDistanceToInsert = INT_MAX;		// TODO:
	int newDistanceFromInsert = INT_MAX;	// TODO:
	int distanceDelta = INT_MAX;			// TODO:

	// Loop over all unplannedTasks
	for (int c : unplannedTasks)
	{
		// Get the earliest and laster possible arrival at the client
		int earliestArrival = params->cli[c].earliestArrival;
		int latestArrival = params->cli[c].latestArrival;

		int bestDistance = INT_MAX;
		std::pair<int, int> bestLocation;

		// Loop over all routes
		for (int r = 0; r < params->nbVehicles; r++)
		{
			// Go to the next route if this route is empty
			if (offspring->chromR[r].empty())
			{
				continue;
			}

			newDistanceFromInsert = params->timeCost.get(c, offspring->chromR[r][0]);
			if (earliestArrival + newDistanceFromInsert < params->cli[offspring->chromR[r][0]].latestArrival)
			{
				distanceDelta = params->timeCost.get(0, c) + newDistanceToInsert
					- params->timeCost.get(0, offspring->chromR[r][0]);
				if (distanceDelta < bestDistance)
				{
					bestDistance = distanceDelta;
					bestLocation = { r, 0 };
				}
			}

			for (int i = 1; i < static_cast<int>(offspring->chromR[r].size()); i++)
			{
				newDistanceToInsert = params->timeCost.get(offspring->chromR[r][i - 1], c);
				newDistanceFromInsert = params->timeCost.get(c, offspring->chromR[r][i]);
				if (params->cli[offspring->chromR[r][i - 1]].earliestArrival + newDistanceToInsert < latestArrival
					&& earliestArrival + newDistanceFromInsert < params->cli[offspring->chromR[r][i]].latestArrival)
				{
					distanceDelta = newDistanceToInsert + newDistanceFromInsert
						- params->timeCost.get(offspring->chromR[r][i - 1], offspring->chromR[r][i]);
					if (distanceDelta < bestDistance)
					{
						bestDistance = distanceDelta;
						bestLocation = { r, i };
					}
				}
			}

			newDistanceToInsert = params->timeCost.get(offspring->chromR[r].back(), c);
			if (params->cli[offspring->chromR[r].back()].earliestArrival + newDistanceToInsert < latestArrival)
			{
				distanceDelta = newDistanceToInsert + params->timeCost.get(c, 0)
					- params->timeCost.get(offspring->chromR[r].back(), 0);
				if (distanceDelta < bestDistance)
				{
					bestDistance = distanceDelta;
					bestLocation = { r, static_cast<int>(offspring->chromR[r].size()) };
				}
			}
		}

		offspring->chromR[bestLocation.first].insert(
			offspring->chromR[bestLocation.first].begin() + bestLocation.second, c);
	}
}

Individual* Genetic::bestOfSREXAndOXCrossovers(std::pair<const Individual*, const Individual*> parents)
{
	// Create two individuals, one with OX and one with SREX
	Individual* offspringOX = crossoverOX(parents);
	Individual* offspringSREX = crossoverSREX(parents);

	//Return the best individual, based on penalizedCost
	return offspringOX->myCostSol.penalizedCost < offspringSREX->myCostSol.penalizedCost
		? offspringOX
		: offspringSREX;
}

Genetic::Genetic(Params* params, Split* split, Population* population, LocalSearch* localSearch) : params(params), split(split), population(population), localSearch(localSearch)
{
	// After initializing the parameters of the Genetic object, also generate new individuals in the array candidateOffsprings
	std::generate(candidateOffsprings.begin(), candidateOffsprings.end(), [&]{ return new Individual(params); });
}

Genetic::~Genetic(void)
{
	// Destruct the Genetic object by deleting all the individuals of the candidateOffsprings
	for (Individual* candidateOffspring : candidateOffsprings)
	{
		delete candidateOffspring;
	}
}
