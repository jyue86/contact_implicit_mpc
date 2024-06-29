#include <iostream>
#include <drake/multibody/parsing/model_directives.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant_config.h>
#include "drake/common/find_resource.h"
#include <drake/geometry/meshcat.h>
#include <drake/multibody/parsing/model_instance_info.h>
#include <drake/multibody/plant/multibody_plant_config_functions.h>
#include <drake/multibody/parsing/process_model_directives.h>

#include "drake/multibody/plant/multibody_plant.h"
#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/systems/framework/system_visitor.h>
#include <drake/systems/framework/context.h>


#define DT 0.01
namespace drake {
namespace multibody {

int run_main() {
    MultibodyPlantConfig config;
    config.time_step = DT;
    config.penetration_allowance = 0.001;
    config.contact_model = "hydroelastic_with_fallback";
    config.contact_surface_representation = "polygon";
    config.discrete_contact_approximation = "tamsi";

    std::shared_ptr<geometry::Meshcat> meshcat = std::make_shared<geometry::Meshcat>();
    systems::DiagramBuilder<double> builder;

    auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);

    Parser parser(&plant, &scene_graph);
    const parsing::ModelDirectives allegro_directive = parsing::LoadModelDirectives("../config/allegro_setup.yaml");
    parsing::ProcessModelDirectives(allegro_directive, &plant, nullptr, &parser);
    plant.Finalize();

    visualization::AddDefaultVisualization(&builder, meshcat);

    std::unique_ptr<systems::Diagram<double>> diagram_ptr = builder.Build();
    std::unique_ptr<systems::Context<double>> context = diagram_ptr->CreateDefaultContext();
    auto& plant_context = plant.GetMyMutableContextFromRoot(context.get());
    meshcat->StartRecording();
    diagram_ptr->ForcedPublish(*context.get());
    for (size_t i = 0 ; i < 100; ++i) {
        context->SetTime(i * DT);
        plant.SetPositions(&plant_context, Eigen::VectorXd::Constant(plant.num_positions(), i / 100.0));
        diagram_ptr->ForcedPublish(*context.get());
    }
    meshcat->StopRecording();
    meshcat->PublishRecording();
    diagram_ptr->ForcedPublish(*context.get());

    int a;
    std::cin >> a;
    return 0;
}


} // namespace multibody
} // namespace drake

int main() {

    return drake::multibody::run_main();
}