// TestFusionCrowd.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "pch.h"
#include <iostream>
#include <fstream>
#include <iterator>
#include <direct.h>
#include <numeric>
#include "TestCases/TradeshowTestCase.h"
#include "TestCases/NeighbourSearchBenchCase.h"
#include "TestCases/ZanlungoCase.h"
#include "TestCases/CrossingTestCase.h"
#include "TestCases/PinholeTestCase.h"
#include "TestCases/TshapedFancyTestCase.h"
#include "TestCases/FsmTestCase.h"
#include "TestCases/StenkaNaStenkuTestCase.h"
#include "TestCases/ITestCase.h"
#include "TestCases/NavGraphTestCase.h"

#include "Export/ComponentId.h"
#include "Export/Export.h"
#include "Util/RecordingSerializer.h"

#include "ThirdParty/date.h"

using namespace TestFusionCrowd;

using time_point = std::chrono::time_point<std::chrono::system_clock>;

void Run(std::shared_ptr<ITestCase> testCase, std::vector<long long> & outMeasurements, time_point & outStartTime)
{

	std::vector<long long> measurements;


	using namespace std::chrono;

	auto steps = testCase->GetStepCount();
	auto sim = testCase->GetSim();
	bool verobseSteps = testCase->VerboseSteps;

	sim->SetIsRecording(testCase->WriteTrajectories);

	outStartTime = system_clock::now();
	for (size_t i = 0; i < steps; i++)
	{
		high_resolution_clock::time_point t1 = high_resolution_clock::now();
		sim->DoStep();

		high_resolution_clock::time_point t2 = high_resolution_clock::now();
		long long duration = duration_cast<microseconds>(t2 - t1).count();
		measurements.push_back(duration);

		if(verobseSteps && i % 10 == 0)
		{
			std::cout << "  Step " << i << "/" << steps
		              << " in " << duration << " microseconds"
		              << std::endl;
		}
	}

	outMeasurements.clear();
	std::copy(measurements.begin(), measurements.end(), std::back_inserter(outMeasurements));

	std::sort(measurements.begin(), measurements.end());
	float avg = std::accumulate(measurements.begin(), measurements.end(), 0.0) / measurements.size();
	std::cout << "  Step time stats:"
		<< " min=" << measurements[0]
		<< " Q1="  << measurements[steps / 4]
		<< " Q2="  << measurements[steps / 2]
		<< " Q3="  << measurements[steps * 3 / 4]
		<< " Q95=" << measurements[steps * 95 / 100]
		<< " max=" << measurements[steps - 1]
		<< " avg"  << avg
		<< std::endl;
	getchar();
}

void WriteToFile(std::shared_ptr<ITestCase> testCase, std::vector<long long> measurements, time_point startTime, std::string folder)
{
	std::string d = date::format("%H%M%S", startTime);
	std::string prefix = folder + "\\" + d + "_" + testCase->GetName();

	if(testCase->WriteTime)
	{
		std::cout << "  1. Writing step times" << std::endl;
		std::ofstream time_measures(prefix + "_step_times_microsec.csv");

		for(auto val : measurements)
		{
			time_measures << val << std::endl;
		}
	}

	if(testCase->WriteTrajectories)
	{
		std::cout << "  2. Writing trajectories" << std::endl;

		std::string filename(prefix + "_trajs.csv");
		auto & rec = testCase->GetSim()->GetRecording();
		FusionCrowd::Recordings::Serialize(rec, filename.c_str(), filename.size());
	}
}

int main()
{
	std::vector<std::shared_ptr<ITestCase>> cases =
	{

		//std::shared_ptr<ITestCase>((ITestCase*) new FsmTestCase(FusionCrowd::ComponentIds::BICYCLE, 100000, 1000, true)),
		std::shared_ptr<ITestCase>((ITestCase*) new TradeshowTestCase(20, 800, true)),
		// std::shared_ptr<ITestCase>((ITestCase*) new ZanlungoCase()),
		//std::shared_ptr<ITestCase>((ITestCase*) new CrossingTestCase(FusionCrowd::ComponentIds::ORCA_ID, 16, 2000, true)),
		// std::shared_ptr<ITestCase>((ITestCase*) new PinholeTestCase(FusionCrowd::ComponentIds::KARAMOUZAS_ID, 2, 100)),
		//std::shared_ptr<ITestCase>((ITestCase*) new TshapedFancyTestCase(FusionCrowd::ComponentIds::BICYCLE, 10, 1000, true)),
		// std::shared_ptr<ITestCase>((ITestCase*) new NavGraphTestCase(100, 1000, true)),
		//std::shared_ptr<ITestCase>((ITestCase*) new StenkaNaStenkuTestCase(10, 1000, true)),
	};

	std::vector<long long> measurements;
	time_point startTime;

	std::string folderName = date::format("%Y%m%d", std::chrono::system_clock::now());
	_mkdir(folderName.c_str());

	for(auto testCase : cases)
	{
		std::cout << "Case " << testCase->GetName() << std::endl
		          << " #agents=" << testCase->GetAgentCount() << ", #step=" << testCase->GetStepCount() << std::endl;

		std::cout << " 1. Setting up..." << std::endl;
		testCase->Pre();
		std::cout << "  Done." << std::endl;

		std::cout << " 2. Running..." << std::endl;
		Run(testCase, measurements, startTime);
		std::cout << "  Done." << std::endl;

		std::cout << " 3. Saving results..." << std::endl;
		WriteToFile(testCase, measurements, startTime, folderName);
		std::cout << "  Done." << std::endl;

		std::cout << " 4. Cleaning up ..." << std::endl;
		testCase->Post();
		std::cout << "  Done." << std::endl;

		std::cout << std::endl << std::endl;
	}

}
