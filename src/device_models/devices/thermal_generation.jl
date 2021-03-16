struct ThermalStandardUCOutages <: PSI.AbstractStandardUnitCommitment end


function PSI.time_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, ThermalStandardUCOutages},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {
    T <: PSY.ThermalGen,
    S <: PM.AbstractPowerModel,
}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    initial_conditions_on =
        PSI.get_initial_conditions(optimization_container, PSI.ICKey(PSI.TimeDurationON, T))
    initial_conditions_off =
        PSI.get_initial_conditions(optimization_container, PSI.ICKey(PSI.TimeDurationOFF, T))
    ini_conds, time_params =
        PSI._get_data_for_tdc(initial_conditions_on, initial_conditions_off, resolution)
    forecast_label = "outage"
    constraint_infos = Vector{DeviceDurationConstraintInfo}
    for (ix, ic) in enumerate(ini_conds[:, 1])
        name = PSI.device_name(ic)
        info = DeviceDurationConstraintInfo(
            name,
            time_params[ix],
            ini_conds[1,:],
            get_time_series(optimization_container, ic.device, forecast_label),
            1.0,
        )
        push!(constraint_infos, info)
    end

    if !(isempty(ini_conds))
        if parameters
            device_duration_parameters_outage!(
                optimization_container,
                time_params,
                ini_conds,
                PSI.make_constraint_name(PSI.DURATION, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                ),
            )
        else
            device_duration_retrospective_outage!(
                optimization_container,
                time_params,
                ini_conds,
                PSI.make_constraint_name(PSI.DURATION, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                ),
            )
        end
    else
        @warn "Data doesn't contain generators with time-up/down limits, consider adjusting your formulation"
    end
    return
end
