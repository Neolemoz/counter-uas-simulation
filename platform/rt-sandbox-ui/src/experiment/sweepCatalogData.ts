import catalogJson from "./fixtures/sweep_catalog_v1.json";
import {
  experimentSweepCatalogSchema,
  type ExperimentSweepCatalog,
} from "./experimentSchema";

export const DEFAULT_SWEEP_CATALOG: ExperimentSweepCatalog =
  experimentSweepCatalogSchema.parse(catalogJson);
