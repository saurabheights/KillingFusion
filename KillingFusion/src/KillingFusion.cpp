//
// Created by Saurabh Khanduja on 22.10.18.
//

#include "KillingFusion.h"

KillingFusion::KillingFusion(DatasetReader datasetReader) : m_datasetReader(datasetReader) {}

void KillingFusion::process() {
  // Create a canonical SDF

  // Save prevSdf to null

  // Save prev2can (previous to canonical displacement field) to null

  // For each file in DatasetReader
  for (int i = 0; i < m_datasetReader.getNumImageFiles(); ++i) {
    // Convert to SDF - currSdf

    // Register currSDF to prevSDF using SDF-2-SDF framework and compure cur2prev displacement field

    // Compute cur2can displacement field by adding cur2prev to prev2can sdfDisplacementField

    // Apply VariationalFramework to minimize the distance between currSdf
  }
}
