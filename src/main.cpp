#include <iostream>
#include <drake/multibody/parsing/model_directives.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant_config.h>
#include "drake/common/find_resource.h"
#include <drake/geometry/meshcat.h>

#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include <drake/systems/framework/diagram_builder.h>

namespace drake {
namespace multibody {


int run_main() {
    MultibodyPlantConfig config;
    config.time_step = 0.001;
    config.penetration_allowance = 0.001;
    config.contact_model = "hydroelastic_with_fallback";
    config.contact_surface_approximation = "polygon";
    config.discrete_contact_approximation = "tamsi";

    geometry::Meshcat meshcat();
    systems::DiagramBuilder<double> builder;
    [plant, scene_graph] = AddMultibodyPlant()

    MultibodyPlant<double> plant;
    geometry::SceneGraph<double> scene_graph;

    std::tie(plant, scene_graph) = AddMultibodyPlant(config, builder);


    parsing::ModelDirectives allegro_directive = parsing::LoadModelDirectives(FindResourceOrThrow("./config/allegro_setup.yaml"));
    std::vector<ModelInstanceInfo> model_instances = parsing::ProcessModelDirectives(allegro_directive, plant, scene_graph);
}


} // namespace multibody
} // namespace drake

int main() {
    std::cout << "Hello, World!" << std::endl;

    
    return 0;
}