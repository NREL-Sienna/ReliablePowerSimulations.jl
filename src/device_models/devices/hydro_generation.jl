struct HydroDispatchRunOfRiverOutage <: PSI.AbstractHydroDispatchFormulation end
struct HydroDispatchReservoirStorageOutage <: PSI.AbstractHydroReservoirFormulation end
struct HydroDispatchPumpedStoragewReservationOutage <: PSI.AbstractHydroReservoirFormulation end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageUpperBoundConstraint},
    U::Type{<:Union{PSI.VariableType, PSI.ExpressionType}},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractPowerModel},
) where {V <: PSY.HydroGen, W <: PSI.AbstractHydroFormulation}
    device_outage_ub_parameter!(
        container,
        T,
        U,
        devices, 
        model,
        X,
    )
    return
end

function PSI.add_constraints!(
    container::PSI.OptimizationContainer,
    T::Type{OutageUpperBoundConstraint},
    devices::IS.FlattenIteratorWrapper{V},
    model::PSI.DeviceModel{V, W},
    X::Type{<:PM.AbstractActivePowerModel},
) where {V <: PSY.HydroPumpedStorage, W <: PSI.AbstractHydroFormulation}

    device_outage_ub_parameter!(
        container,
        T,
        PSI.ActivePowerInVariable,
        devices, 
        model,
        X,
    )
    device_outage_ub_parameter!(
        container,
        T,
        PSI.ActivePowerOutVariable,
        devices, 
        model,
        X,
    )

    return
end
