struct ThermalStandardUCOutages <: PSI.AbstractStandardUnitCommitment end

function PSI.time_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, ThermalStandardUCOutages},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {T <: PSY.ThermalGen, S <: PM.AbstractPowerModel}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    initial_conditions_on =
        PSI.get_initial_conditions(optimization_container, PSI.ICKey(PSI.TimeDurationON, T))
    initial_conditions_off = PSI.get_initial_conditions(
        optimization_container,
        PSI.ICKey(PSI.TimeDurationOFF, T),
    )
    ini_conds, time_params =
        PSI._get_data_for_tdc(initial_conditions_on, initial_conditions_off, resolution)
    forecast_label = "outage"
    constraint_infos = Vector{DeviceDurationConstraintInfo}()
    for (ix, ic) in enumerate(ini_conds[:, 1])
        name = PSI.device_name(ic)
        info = DeviceDurationConstraintInfo(
            name,
            time_params[ix],
            Tuple(ini_conds[ix, :]),
            1.0,
            PSI.get_time_series(optimization_container, ic.device, forecast_label),
        )
        push!(constraint_infos, info)
    end

    if !(isempty(ini_conds))
        if parameters
            device_duration_parameters_outage!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(PSI.DURATION, T),
                (
                    PSI.make_variable_name(PSI.OnVariable, T),
                    PSI.make_variable_name(PSI.StartVariable, T),
                    PSI.make_variable_name(PSI.StopVariable, T),
                ),
                PSI.UpdateRef{T}(OUTAGE, forecast_label),
            )
        else
            device_duration_look_ahead_outage!(
                optimization_container,
                constraint_infos,
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

function outage_constraints!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    model::PSI.DeviceModel{T, ThermalStandardUCOutages},
    ::Type{S},
    feedforward::Union{Nothing, PSI.AbstractAffectFeedForward},
) where {T <: PSY.ThermalGen, S <: PM.AbstractPowerModel}
    parameters = PSI.model_has_parameters(optimization_container)
    resolution = PSI.model_resolution(optimization_container)
    initial_conditions =
        PSI.get_initial_conditions(optimization_container, PSI.ICKey(OutageStatus, T))
    forecast_label = "outage"
    constraint_infos = Vector{DeviceOutageConstraintInfo}()
    for (ix, ic) in enumerate(initial_conditions)
        name = PSI.device_name(ic)
        info = DeviceOutageConstraintInfo(
            name,
            ic,
            1.0,
            PSI.get_time_series(optimization_container, ic.device, forecast_label),
        )
        push!(constraint_infos, info)
    end

    if !(isempty(initial_conditions))
        if parameters
            # device_outage!(
            #     optimization_container,
            #     constraint_infos,
            #     PSI.make_constraint_name(OUTAGE, T),
            #     (
            #         PSI.make_variable_name(PSI.StartVariable, T),
            #         PSI.make_variable_name(PSI.StopVariable, T),
            #     ),
            #     PSI.UpdateRef{T}(OUTAGE, forecast_label),
            # )
            device_outage_ub_parameter!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(OUTAGE, T),
                PSI.make_variable_name(PSI.ON, T),
                PSI.UpdateRef{T}(OUTAGE, forecast_label),
            )
        else
            # device_outage_parameter!(
            #     optimization_container,
            #     constraint_infos,
            #     PSI.make_constraint_name(PSI.DURATION, T),
            #     (
            #         PSI.make_variable_name(PSI.StartVariable, T),
            #         PSI.make_variable_name(PSI.StopVariable, T),
            #     ),
            # )
            device_outage_ub!(
                optimization_container,
                constraint_infos,
                PSI.make_constraint_name(OUTAGE, T),
                PSI.make_variable_name(PSI.ON, T),
            )
        end
    else
        @warn "Data doesn't contain generators with initial condition for outage status, consider adjusting your formulation"
    end

    return
end

function PSI.initial_conditions!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::ThermalStandardUCOutages,
) where {T <: PSY.ThermalGen}
    PSI.status_initial_condition!(optimization_container, devices, formulation)
    PSI.output_initial_condition!(optimization_container, devices, formulation)
    PSI.duration_initial_condition!(optimization_container, devices, formulation)
    outage_status_initial_condition!(optimization_container, devices, formulation)
    return
end

function outage_status_initial_condition!(
    optimization_container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    ::ThermalStandardUCOutages,
) where {T <: PSY.ThermalGen}
    PSI._make_initial_conditions!(
        optimization_container,
        devices,
        ThermalStandardUCOutages(),
        nothing,
        PSI.ICKey(OutageStatus, T),
        _make_initial_condition_outage_status,
        _get_outage_initial_value,
    )

    return
end
