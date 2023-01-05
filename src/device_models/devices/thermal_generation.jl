abstract type AbstractThermalOutageDispatchFormulation <: PSI.AbstractThermalDispatchFormulation end
abstract type AbstractThermalOutageCommitmentFormulation <: PSI.AbstractStandardUnitCommitment end
struct ThermalStandardUCOutages <: AbstractThermalOutageCommitmentFormulation end
struct ThermalBasicUCOutages <: AbstractThermalOutageCommitmentFormulation end
struct ThermalDispatchOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalRampLimitedOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalNoMinOutages <: AbstractThermalOutageDispatchFormulation end
struct ThermalNoMinRampLimitedOutages <: AbstractThermalOutageDispatchFormulation end

############## AuxiliaryOnVariable, ThermalGen ####################
PSI.get_variable_binary(::AuxiliaryOnVariable, ::Type{<:PSY.ThermalGen}, _) = false
# PSI.get_variable_initial_value(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) =
#     PSY.get_status(d) ? 1.0 : 0.0

PSI.get_variable_lower_bound(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = 0.0
PSI.get_variable_upper_bound(::AuxiliaryOnVariable, d::PSY.ThermalGen, _) = 1.0

######## CONSTRAINTS ############
PSI.initial_condition_default(::InitialOutageStatus, d::PSY.ThermalGen, ::PSI.AbstractThermalFormulation) = PSY.get_status(d) ? 1 : 0
PSI.initial_condition_variable(::InitialOutageStatus, d::PSY.ThermalGen, ::PSI.AbstractThermalFormulation) = PSI.OnVariable()


function _get_data_for_tdc_outages(
    container::PSI.OptimizationContainer,
    ::Type{T},
) where {T <: PSY.ThermalGen}
    resolution = PSI.get_resolution(container)
    steps_per_hour = 60 / Dates.value(Dates.Minute(resolution))
    fraction_of_hour = 1 / steps_per_hour
    initial_conditions_on = PSI.get_initial_condition(container, PSI.InitialTimeDurationOn(), T)
    initial_conditions_off = PSI.get_initial_condition(container, PSI.InitialTimeDurationOff(), T)
    initial_conditions_outage = PSI.get_initial_condition(container, InitialOutageStatus(), T)

    lenght_devices_on = length(initial_conditions_on)
    lenght_devices_off = length(initial_conditions_off)
    lenght_devices_outage = length(initial_conditions_outage)
    @assert lenght_devices_off == lenght_devices_on == lenght_devices_outage
    data = Vector{DeviceDurationConstraintInfo}(undef, lenght_devices_on)
    idx = 0
    for (ix, ic) in enumerate(initial_conditions_on)
        g = ic.component
        @assert g == initial_conditions_off[ix].component
        @assert g == initial_conditions_outage[ix].component

        time_limits = PSY.get_time_limits(g)
        name = PSY.get_name(g)
        if !(time_limits === nothing)
            if (time_limits.up <= fraction_of_hour) & (time_limits.down <= fraction_of_hour)
                @debug "Generator $(name) has a nonbinding time limits. Constraints Skipped"
                continue
            else
                idx += 1
            end
            up_val = round(time_limits.up * steps_per_hour, RoundUp)
            down_val = round(time_limits.down * steps_per_hour, RoundUp)
            data[idx] = DeviceDurationConstraintInfo(
                name,
                (up = up_val, down = down_val),
                (ic, initial_conditions_off[ix]),
                initial_conditions_outage[ix],
                1.0,
                PSI.get_time_series(container, g, "outage"),
            )
        end
    end
    if idx < lenght_devices_on
        deleteat!(data, (idx + 1):lenght_devices_on)
    end
    return data
end


function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{OutageTimeConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    ::PSI.DeviceModel{U, V},
    ::Type{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    # Use getter functions that don't require creating the keys here
    parameters = PSI.built_for_recurrent_solves(container)
    constraint_infos = _get_data_for_tdc_outages(container, U)    
    if parameters
        device_duration_parameters_outage!(
            container,
            OutageTimeConstraint,
            constraint_infos,
            devices,
        )
    else
        device_duration_look_ahead_outage!(
            container,
            OutageTimeConstraint(),
            constraint_infos,
            devices,
        )
    end
    return
end

function _get_data_for_rocc_outage(
    container::PSI.OptimizationContainer,
    ::Type{T},
) where {T <: PSY.ThermalGen}
    resolution = PSI.get_resolution(container)
    if resolution > Dates.Minute(1)
        minutes_per_period = Dates.value(Dates.Minute(resolution))
    else
        @warn("Not all formulations support under 1-minute resolutions. Exercise caution.")
        minutes_per_period = Dates.value(Dates.Second(resolution)) / 60
    end

    initial_conditions_power =
        PSI.get_initial_condition(container, PSI.DevicePower(), T)
    initial_conditions_outage =
        PSI.get_initial_condition(container, InitialOutageStatus(), T)
    lenght_devices_power = length(initial_conditions_power)
    lenght_devices_outage = length(initial_conditions_outage)
    @assert lenght_devices_power == lenght_devices_outage
    data = Vector{DeviceOutageRampConstraintInfo}(undef, lenght_devices_power)
    idx = 0
    for (ix, ic) in enumerate(initial_conditions_power)
        g = ic.component
        @assert g == initial_conditions_outage[ix].component
        name = PSY.get_name(g)
        ramp_limits = PSY.get_ramp_limits(g)
        if !(ramp_limits === nothing)
            p_lims = PSY.get_active_power_limits(g)
            max_rate = abs(p_lims.min - p_lims.max) / minutes_per_period
            if (ramp_limits.up >= max_rate) & (ramp_limits.down >= max_rate)
                @debug "Generator $(name) has a nonbinding ramp limits. Constraints Skipped"
                continue
            else
                idx += 1
            end
            ramp = (
                up = ramp_limits.up * minutes_per_period,
                down = ramp_limits.down * minutes_per_period,
            )
            data[idx] = DeviceOutageRampConstraintInfo(
                name,
                p_lims,
                ic,
                initial_conditions_outage[ix],
                ramp,
            )
        end
    end
    if idx < lenght_devices_power
        deleteat!(data, (idx + 1):lenght_devices_power)
    end
    return data
end


function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{OutageRampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::PSI.DeviceModel{U, V},
    ::Type{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    data = _get_data_for_rocc_outage(container, U)
    if !isempty(data)
        # Here goes the reactive power ramp limits when versions for AC and DC are added
        # ?NG is this still needed
        # for r in data
        #     PSI.add_device_services!(r, r.ic_power.device, model)
        # end
        device_mixedinteger_rateofchange_outages!(
            container,
            OutageRampConstraint,
            data,
            devices,
            model
        )
    else
        @warn "Data doesn't contain generators with ramp limits, consider adjusting your formulation"
    end
    return
end


function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    ::Type{OutageRampConstraint},
    devices::IS.FlattenIteratorWrapper{U},
    model::PSI.DeviceModel{U, V},
    ::Type{<:PM.AbstractPowerModel},
) where {U <: PSY.ThermalGen, V <: PSI.AbstractThermalDispatchFormulation}
    data = _get_data_for_rocc_outage(container, U)
    if !isempty(data)
        # Here goes the reactive power ramp limits when versions for AC and DC are added
        # ?NG is this still needed
        # for r in data
        #     PSI.add_device_services!(r, r.ic_power.device, model)
        # end
        device_linear_rateofchange_outages!(
            container,
            OutageRampConstraint,
            data,
            devices,
            model
        )
    else
        @warn "Data doesn't contain generators with ramp limits, consider adjusting your formulation"
    end
    return
end


function PSI.get_default_time_series_names(
    ::Type{<:PSY.ThermalGen},
    ::Type{<:Union{AbstractThermalOutageDispatchFormulation, AbstractThermalOutageCommitmentFormulation}},
)
    return Dict{Type{<:PSI.TimeSeriesParameter}, String}(
        OutageTimeSeriesParameter => "outage",
    )
end

PSI.get_multiplier_value(::PSI.TimeSeriesParameter, d::PSY.ThermalGen, :: AbstractThermalOutageDispatchFormulation) = PSY.get_max_active_power(d)
PSI.get_multiplier_value(::PSI.TimeSeriesParameter, d::PSY.ThermalGen, :: AbstractThermalOutageCommitmentFormulation) = PSY.get_max_active_power(d)


function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageCommitmentConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}

    parameters = PSI.built_for_recurrent_solves(container)
    resolution = PSI.get_resolution(container)
    initial_conditions =
        PSI.get_initial_condition(container, InitialOutageStatus(), V)
    forecast_label = "outage"
    constraint_infos = Vector{DeviceOutageConstraintInfo}()
    for ic in initial_conditions
        device = ic.component
        name = PSI.get_name(device)
        info = DeviceOutageConstraintInfo(
            name,
            ic,
            1.0,
            PSI.get_time_series(container, device, forecast_label),
        )
        push!(constraint_infos, info)
    end

    if !(isempty(devices))
        device_outage_parameter!(
            container,
            constraint_infos,
            T, 
            devices,
        )
    else
        @warn "Data doesn't contain generators of type $T, consider adjusting your formulation"
    end

    return
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageUpperBoundConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: AbstractThermalOutageDispatchFormulation}

    parameters = PSI.built_for_recurrent_solves(container)
    resolution = PSI.get_resolution(container)
    initial_conditions =
        PSI.get_initial_condition(container, InitialOutageStatus(), V)
    forecast_label = "outage"
    constraint_infos = Vector{DeviceOutageConstraintInfo}()
    for ic in initial_conditions
        device = ic.component
        name = PSI.get_name(device)
        info = DeviceOutageConstraintInfo(
            name,
            ic,
            1.0,
            PSI.get_time_series(container, device, forecast_label),
        )
        push!(constraint_infos, info)
    end

    if !(isempty(devices))
        device_outage_ub_parameter!(
            container,
            constraint_infos,
            T, 
            devices, 
            PSI.ActivePowerVariable()
        )
    else
        @warn "Data doesn't contain generators of type $T, consider adjusting your formulation"
    end

    return
end


function PSI.initial_conditions!(
    container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: AbstractThermalOutageDispatchFormulation}
    PSI.add_initial_condition!(container, devices, formulation, PSI.DevicePower())
    PSI.add_initial_condition!(container, devices, formulation, InitialOutageStatus())
    return
end

function PSI.initial_conditions!(
    container::PSI.OptimizationContainer,
    devices::IS.FlattenIteratorWrapper{T},
    formulation::D,
) where {T <: PSY.ThermalGen, D <: Union{ThermalStandardUCOutages, ThermalBasicUCOutages}}
    PSI.add_initial_condition!(container, devices, formulation, PSI.DeviceStatus())
    PSI.add_initial_condition!(container, devices, formulation, PSI.DevicePower())
    PSI.add_initial_condition!(container, devices, formulation, PSI.InitialTimeDurationOn())
    PSI.add_initial_condition!(container, devices, formulation, PSI.InitialTimeDurationOff())
    PSI.add_initial_condition!(container, devices, formulation, InitialOutageStatus())
    return
end


"""
Min and max active power limits of generators for thermal dispatch no minimum formulations
"""
function PSI.get_min_max_limits(
    device,
    ::Type{PSI.ActivePowerVariableLimitsConstraint},
    ::Type{S},
) where {S <: Union{ThermalNoMinOutages, ThermalNoMinRampLimitedOutages}}
    return (min=0.0, max=PSY.get_active_power_limits(device).max)
end

PSI.get_variable_lower_bound(::PSI.ActivePowerVariable, d::PSY.ThermalGen, ::Union{ThermalNoMinOutages, ThermalNoMinRampLimitedOutages}) = 0.0


PSI._get_initial_condition_type(
    ::Type{OutageRampConstraint},
    ::Type{<:PSY.ThermalGen},
    ::Type{<:AbstractThermalOutageCommitmentFormulation},
) = DevicePower


function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{<:PSI.PowerVariableLimitsConstraint},
    U::Type{<:Union{PSI.VariableType, PSI.ExpressionType}},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.ThermalGen, W <: AbstractThermalOutageDispatchFormulation}
    if !has_semicontinuous_outage_feedforward(model, U)
        PSI.add_range_constraints!(container, T, U, devices, model, X)
    end
    return
end