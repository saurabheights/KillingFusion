//
// Created by Saurabh Khanduja on 18.10.18.
//
#include <string>

#include "KillingFusion.h"
#include "DatasetReader.h"
#include "config.h"

int main(int argc, char *argv[]) {
  DatasetReader datasetReader(DATA_DIR, SNOOPY);
  KillingFusion fusion(datasetReader);
  fusion.process();
}

