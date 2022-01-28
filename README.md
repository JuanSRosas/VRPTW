# Ambiente

O código escrito na linguagem Julia foi executado com  instâncias para 25, 50 e 100 clientes,utilizando o pacote de otimização de código aberto JuMP e o pacote de otimização proprietário IBM-CPLEX 12.10, utilizando uma licença para estudantes. Foi imposto um limite de duas horas (7200 segundos) para os testes.

# VRPTW


# Instancias de Solomom

As instâncias de Solomon são divididas em 3 tipos, tipo R, C e RC que subdividem (R1 e R2, C1 e C2, RC1 eRC2), As instâncias R1 e R2 incluem locais de clientes aleatórios, as instâncias C1 e C2 contêm locais de clientes agrupados e as instâncias RC1 e RC2 contêm uma mistura de locais de clientes aleatórios e agrupados. As janelas de tempo nas instâncias R1, C1 e RC1 tendem a ser estreitas, enquanto nas instâncias R2, C2 e RC2, elas tendem a ser maiores. 

As coordenadas do cliente são idênticas para todos os problemas dentro de um tipo (seja o tipo R, C e RC). Os problemas vão se diferenciar em relação à largura das janelas de tempo. Alguns têm janelas de tempo muito apertadas, enquanto outros têm janelas de tempo que dificilmente são restritivas. Em termos de densidade de janela de tempo, ou seja, a porcentagem de clientes com janelas de tempo variam entre 25, 50, 75 e 100%. Para cada uma das instancia foram criadas instancias menores foram criados considerando apenas os primeiros 25 ou 50 clientes.

