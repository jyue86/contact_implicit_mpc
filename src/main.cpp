#include <iostream>
#include <drake/multibody/parsing/model_directives.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant_config.h>
#include "drake/common/find_resource.h"
#include <drake/geometry/meshcat.h>
#include <drake/multibody/parsing/model_instance_info.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>
#include <drake/multibody/parsing/process_model_directives.h>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/parsing/parser.h>


#define DT 0.001
namespace drake {
namespace multibody {

int run_main() {
    MultibodyPlantConfig config;
    config.time_step = DT;
    config.penetration_allowance = 0.001;
    config.contact_model = "hydroelastic_with_fallback";
    config.contact_surface_representation = "polygon";
    config.discrete_contact_approximation = "tamsi";

    geometry::Meshcat meshcat;
    systems::DiagramBuilder<double> builder;

    auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);

    Parser parser(&plant, &scene_graph);
    const parsing::ModelDirectives allegro_directive = parsing::LoadModelDirectives("../config/allegro_setup.yaml");
    parsing::ProcessModelDirectives(allegro_directive, &plant, nullptr, &parser);


    return 0;
}


} // namespace multibody
} // namespace drake

int main() {

    return drake::multibody::run_main();
}