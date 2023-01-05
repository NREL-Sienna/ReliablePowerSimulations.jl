struct RenewableOutageDispatch <: PSI.AbstractRenewableDispatchFormulation end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageUpperBoundConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractActivePowerModel},
) where {V <: PSY.RenewableGen, W <: RenewableOutageDispatch}
    parameters = PSI.built_for_recurrent_solves(container)
    resolution = PSI.get_resolution(container)

    forecast_label = "outage"
    constraint_infos = Vector{DeviceOutageConstraintInfo}()
    for d in devices
        name = PSY.get_name(d)
        info = DeviceOutageConstraintInfo(
            name,
            nothing,
            1.0,
            PSI.get_time_series(container, d, forecast_label),
        )
        push!(constraint_infos, info)
    end

    if !(isempty(devices))
        device_outage_ub_parameter!(
            container,
            constraint_infos,
            T, 
            devices, 
            U
        )
    else
        @warn "Data doesn't contain generators of type $T, consider adjusting your formulation"
    end

    return
end