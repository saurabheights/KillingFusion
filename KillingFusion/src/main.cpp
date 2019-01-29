//
// Created by Saurabh Khanduja on 18.10.18.
//
#include <string>

#include "KillingFusion.h"
#include "DatasetReader.h"
#include "config.h"

int main(int argc, char ** argv) {
  DatasetReader datasetReader(DATA_DIR);
  KillingFusion fusion(datasetReader);
  DisplacementField::testJacobian();
  DisplacementField::testKillingEnergy();
  // fusion.processTest(1);
  // fusion.processTest(2);
  // fusion.processTest(3);
  // fusion.process();
}

