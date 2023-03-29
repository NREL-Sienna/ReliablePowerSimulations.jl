function PSI._get_initial_conditions_value(
    ::Vector{T},
    component::W,
    ::U,
    ::V,
    container::PSI.OptimizationContainer,
) where {
    T <: PSI.InitialCondition{InitialOutageStatus, Float64},
    V <: PSI.AbstractDeviceFormulation,
    W <: PSY.Component,
    U <: InitialOutageStatus,
}
    ic_data = PSI.get_initial_conditions_data(container)
    val = PSI.initial_condition_default(U(), component, V())
    @debug "Device $(PSY.get_name(component)) initialized DeviceStatus as $var_type" _group =
        PSI.LOG_GROUP_BUILD_INITIAL_CONDITIONS
    return T(component, val)
end

function PSI._get_initial_conditions_value(
    ::Vector{T},
    component::W,
    ::U,
    ::V,
    container::PSI.OptimizationContainer,
) where {
    T <: PSI.InitialCondition{InitialOutageStatus, PJ.ParameterRef},
    V <: PSI.AbstractDeviceFormulation,
    W <: PSY.Component,
    U <: InitialOutageStatus,
}
    ic_data = PSI.get_initial_conditions_data(container)
    val = PSI.initial_condition_default(U(), component, V())
    @debug "Device $(PSY.get_name(component)) initialized DeviceStatus as $var_type" _group =
        PSI.LOG_GROUP_BUILD_INITIAL_CONDITIONS
    return T(component, PSI.add_jump_parameter(PSI.get_jump_model(container), val))
end

# function PSI.update_initial_conditions!(
#     ics::Vector{T},
#     state::PSI.SimulationState,
#     ::Dates.Millisecond,
# ) where {
#     T <: PSI.InitialCondition{InitialOutageStatus, S},
# } where {S <: Union{Float64, PJ.ParameterRef}}
#     for ic in ics
#         var_val = PSI.get_system_state_value(
#             state,
#             AuxiliaryOnVariable(),
#             PSI.get_component_type(ic),
#         )
#         PSI.set_ic_quantity!(ic, var_val[PSI.get_component_name(ic)])
#     end
#     return
# end




# why using AuxiliaryOnVariable to update InitialOutageStatus instead of outage time series?
function PSI.update_initial_conditions!(
    ics::Vector{T},
    state::PSI.SimulationState,
    ::Dates.Millisecond,
) where {
    T <: PSI.InitialCondition{InitialOutageStatus, S},
} where {S <: Union{Float64, PJ.ParameterRef}}
    for ic in ics
        if PSI.get_component_type(ic) == PSY.ThermalStandard
            var_val = PSI.get_system_state_value(
                state,
                AuxiliaryOnVariable(),
                PSI.get_component_type(ic),
            )
        else    # this should be ThermalFastStartSIIP, but that is defined in EMISAgentSimulation.jl, which is dependent on RPSI, probably won't work?
            var_val = PSI.get_system_state_value(
                state,
                PSI.OnVariable(),
                PSI.get_component_type(ic),
            )
        end
        PSI.set_ic_quantity!(ic, var_val[PSI.get_component_name(ic)])
    end
    return
end




# over-write update DeviceStatus initial condition in PSI
function PSI.update_initial_conditions!(
    ics::Vector{T},
    state::PSI.SimulationState,
    ::Dates.Millisecond,
) where {
    T <: PSI.InitialCondition{PSI.DeviceStatus, S},
} where {S <: Union{Float64, PJ.ParameterRef}}
    for ic in ics
        if PSI.get_component_type(ic) == PSY.ThermalStandard
            var_val = PSI.get_system_state_value(
                state,
                AuxiliaryOnVariable(),
                PSI.get_component_type(ic),
            )
        else    # this should be ThermalFastStartSIIP, but that is defined in EMISAgentSimulation.jl, which is dependent on RPSI, probably won't work?
            var_val = PSI.get_system_state_value(
                state,
                PSI.OnVariable(),
                PSI.get_component_type(ic),
            )
        end
        PSI.set_ic_quantity!(ic, var_val[PSI.get_component_name(ic)])
    end
    return
end
