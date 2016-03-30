import random

def generatePopulation():
    population = ["100011010111", "11010111010"]

    return population

def performCrossover(parent1, parent2):
    # Crossover rate can be 0 or 100.
    crossPoint = random.randint(0, len(parent1) - 1)
    child = parent1[:crossPoint] + parent2[crossPoint:]

    return child

def main():
    pop = generatePopulation()
    newPop = []

    for i in range(0, len(pop) - 1):
        parent1 = pop[i]
        parent2 = pop[i + 1]

        child = performCrossover(parent1, parent2)
        newPop.append(child)

        print "Child: ", child

main()
