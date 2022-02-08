import Base.show, Base.print

mutable struct Vertex
   id_vertex::Int
   pos_x::Float64
   pos_y::Float64
   service_time::Int
   demand::Int
   start_tw::Int
   end_tw::Int
end

# Directed graph
mutable struct InputGraph
   V′::Array{Vertex} # set of vertices
   A::Array{Tuple{Int64,Int64}} # set of edges
   cost::Dict{Tuple{Int64,Int64},Float64} # cost for each arc
   time::Dict{Tuple{Int64,Int64},Float64} # time for each arc
end

mutable struct DataVRPTW
   n::Int
   G′::InputGraph
   Q::Float64 # vehicle capacity
   K::Int #num vehicles available
end

# Euclidian distance
function distance(v1::Vertex, v2::Vertex)
   x_sq = (v1.pos_x - v2.pos_x)^2
   y_sq = (v1.pos_y - v2.pos_y)^2
   return floor(sqrt(x_sq + y_sq)*10)/10
end

function readVRPTWData(path_file::String)
   # STEP 1 : pushing data in a vector.
   data = Array{Any,1}()
   open(path_file) do file
      for line in eachline(file)
         if findfirst("CUST", line) != nothing || findfirst("VEH", line) != nothing || 
	    findfirst("NUMBER", line) != nothing
            continue
         end
         for peaceofdata in split(line)
            push!(data, String(peaceofdata))
         end
      end
   end

   n = div(length(data) - 3, 7) - 1
    #n = div(length(data) - 3, 6) - 1 #thailson
   K = parse(Int, data[2])
   Q = parse(Float64, data[3])

   vertices = Vertex[] 
   for i in 0:n
      offset = 3 + i*7
      x = parse(Float64, data[offset + 2])     
      y = parse(Float64, data[offset + 3])     
      d = parse(Int, data[offset + 4])     
      l = parse(Int, data[offset + 5])     
      u = parse(Int, data[offset + 6])     
      s = parse(Int, data[offset + 7])     
      push!(vertices, Vertex(i, x, y, s, d, l, u))
   end
  
  push!(vertices, vertices[1])  
    
   A = Tuple{Int64,Int64}[]
   cost = Dict{Tuple{Int64,Int64},Float64}()
   time = Dict{Tuple{Int64,Int64},Float64}()

   function add_arc!(i, j)
      push!(A, (i,j)) 
      cost[(i,j)] = distance(vertices[i + 1], vertices[j + 1])  
      time[(i,j)] = distance(vertices[i + 1], vertices[j + 1]) + vertices[i + 1].service_time 
   end
    
   for i in 1:n
      #arc from depot
      add_arc!(0, i)
      #arc to depot
      #add_arc!(i, 0)  
      add_arc!(i, n+1)  #thailson
      for j in 1:n
         if (i != j) 
            add_arc!(i, j)
         end
      end
   end

   add_arc!(0, n+1)   
    
   DataVRPTW(n, InputGraph(vertices, A, cost, time), Q, K)
end

arcs(data::DataVRPTW) = data.G′.A # return set of arcs

#função retorna custo de um arco
function c(data,a) 
    if(a[1] == a[2])
      return 10000000
    elseif !(haskey(data.G′.cost, a)) 
      return 10000000
   end
   return data.G′.cost[a]  
end    

#função retorna custo do tempo de um arco
function t(data,a) 
   if(a[1] == a[2])
        return 10000000
   end
   if !(haskey(data.G′.time, a)) 
        return 10000000
   end
   return data.G′.time[a] 
end   

#tempoarco(i,j) = sqrt((data.G′.V′[i].pos_x-data.G′.V′[j].pos_x)^2+(data.G′.V′[i].pos_y-data.G′.V′[j].pos_y)^2)

n(data::DataVRPTW) = data.n # numeros de entregas
d(data::DataVRPTW, i) = data.G′.V′[i+1].demand # return demand of i
s(data::DataVRPTW, i) = data.G′.V′[i+1].service_time # return service time of i
l(data::DataVRPTW, i) = data.G′.V′[i+1].start_tw #inicio do TW
u(data::DataVRPTW, i) = data.G′.V′[i+1].end_tw #fim do Tw
veh_capacity(data::DataVRPTW) = Int(data.Q) #capacidade do veiculo

function lowerBoundNbVehicles(data::DataVRPTW) #limite inferior
   return 1
end

function k(data::DataVRPTW) #limite superior
   return data.K
end



#artigo The Vehicle Routing Problem with Time Windows
using JuMP, CPLEX
arq= "C101.txt" #
println("Início execução - Lendo arquivo ", arq)
data = readVRPTWData(arq)
num=n(data)
nv=k(data)

println("qtde demandas: ", num)
println("qtde veículos: ", nv)

#variaveis 
modelo = Model(with_optimizer(CPLEX.Optimizer))
@variable(modelo,x[0:n(data)+1,0:n(data)+1,1:k(data)],Bin) #Cijk
@variable(modelo,T[0:n(data)+1,1:k(data)],lower_bound = 0) #Tik

# Função objetivo
@objective(modelo, Min ,sum(c(data,(i,j))*x[i,j,k] for i=0:n(data)+1,j=0:n(data)+1, k=1:k(data)))
    
# Usar veículo uma vez

for i in 1:n(data)
    @constraint(modelo, sum(x[i,j,k] for j=0:n(data)+1, k=1:k(data)) == 1)
end
    
# Tudo que chega no vértice sai dele

for i in 1:n(data)
    for k in 1:k(data)
        @constraint(modelo, sum(x[j,i,k] for j =0:n(data)+1) == sum(x[i,j,k] for j =0:n(data)+1))
        @constraint(modelo, x[i,i,k] == 0 )
    end
end
 
# Em cada rota -> sai uma vez do depósito e volta uma vez

for k in 1:k(data)
    @constraint(modelo,sum(x[0,j,k] for j=1:n(data)+1) == 1)
    @constraint(modelo,sum(x[i,n(data)+1,k] for i=0:n(data)) == 1)
end

# Restrição de Capacidade

for k in 1:k(data)
    @constraint(modelo, sum(x[i,j,k]*d(data,i) for i=1:n(data),j=1:n(data)+1) <= veh_capacity(data))
end

# Eliminando Subrota

for k in 1:k(data)
    for a in arcs(data)
        i = a[1]
        j = a[2]
        if(i != j)
            M = max(u(data,i)+t(data, (i,j))-l(data,j),0)
           @constraint(modelo, T[i,k] + t(data, (i,j)) - T[j,k] <=  ((1-x[i,j,k])*M))   
        end 
    end
end


# - Time Windows

for k in 1:k(data)
    for i in 0:n(data)
        @constraint(modelo, T[i,k] >= l(data,i))
        @constraint(modelo, T[i,k] <= u(data,i))
    end
end


optimize!(modelo) 
@show solution_summary(modelo, verbose=true)

#println(modelo)
#has_values(modelo)
#relative_gap(modelo)
#termination_status(modelo)