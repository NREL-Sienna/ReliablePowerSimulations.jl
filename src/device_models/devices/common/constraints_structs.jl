struct DeviceDurationConstraintInfo <: PSI.AbstractStartConstraintInfo
    component_name::String
    duration_data::PSI.UpDown
    initial_duration::Tuple{PSI.InitialCondition, PSI.InitialCondition}
    multiplier::Float64
    timeseries::Vector{Float64}
end

struct DeviceOutageConstraintInfo <: PSI.AbstractStartConstraintInfo
    component_name::String
    initial_condition::Union{Nothing, PSI.InitialCondition}
    multiplier::Float64
    timeseries::Vector{Float64}
end
