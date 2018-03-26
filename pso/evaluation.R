
# Parameters
waypoints = seq (2, 10)
runs = seq (1, 100)


#### Doorways. Experiment ?: (93, 56) --> (62, 148)

# Load data 
df <- read.csv ("/home/krell/Downloads/experimentTable_Doorways.csv", header = FALSE)

# Separate out the obstacle table
obstacles <- df[df$V5 == -1,]
df <- df[df$V5 != -1, ]


getByWaypoint <- function (odf, path, waypoint) {
  odf[odf$V3 == waypoint, ]
  
}

# Table to store feasability percentage
feasability <- data.frame(waypoint=character(),
           feasableCount=integer(),
           feasableProportion=integer(),
           stringsAsFactors=FALSE)
colnames (feasability) <- c ("Waypoints", "NumFeasible", "PropFeasible")


for (i in waypoints) {
  o <- getByWaypoint (obstacles, 1, i)
  numFeasible <- length (which (o$V6 == 0))
  propFeasible <- numFeasible / length (runs)
  if (is.infinite(propFeasible)) {
    propFeasible <- 0
  }
  
  feasability[nrow(feasability) + 1,] = list(i, numFeasible, propFeasible)
  
}


library(forcats) # Used to enforce order of bar chart X axis

# Inside bars
ggplot(data=feasability, aes(x=Waypoints, y=PropFeasible)) +
  geom_bar(stat="identity", fill="steelblue")+
  geom_text(aes(label=PropFeasible), vjust=1.6, color="white", size=3.5)+
  theme_minimal()  + aes(x = fct_inorder(Waypoints)) + labs (y = "Proportion of Collision-Free Paths", x = "Waypoints")



# Separate into feasible/infeasible data frames

##############
## Feasible ##
##############
getFeasibleIdx <- function (df, path, waypoint){
  o <- getByWaypoint (obstacles, path, waypoint)
  which (o$V6 == 0)
}
feasibleRunIdxs <- list ()
for (w in waypoints) {
  feasibleRunIdxs[[w]] <- getFeasibleIdx (obstacles, 1, w)  
}


dfFeasible <- data.frame (V1=character(), V2=character(), V3=character(), V4=character(), V5=character(), V6=character())
for (w in waypoints){
    dfw <- df[df$V3 == w, ]
    dfwF <- dfw[dfw$V4 %in% feasibleRunIdxs[[w]], ]
    if (length (dfwF) != 0){
      dfFeasible <- rbind (dfFeasible, dfwF)
    }
}

# Get a specific run's data
extractRun <- function (df, path, waypoint, run) {
  w <- getByWaypoint (df, path, waypoint)
  
  r <- w[w$V4 == run, ]
  if (length (r) > 0){
    as.numeric (r$V6)
  }
}

# Get a list where each elem is a table of runs and steps for each waypoint
convergence = vector("list", length (waypoints)) 
for (way in waypoints) {
  convergence[[way]] <- sapply (X = feasibleRunIdxs[[way]], FUN = function (x) {extractRun (dfFeasible, 1, way, x)})
}

# Get the avg run for a waypoint
avgRun <- function (cdf, waypoint){
  w <- cdf[[waypoint]]
  if (length (w) > 0) {
    apply (X = w, MARGIN = 1, FUN = mean)
  }
}
convergence_avg = vector("list", length (waypoints))
for (way in waypoints) {
  convergence_avg[[way]] <- avgRun (convergence, way)
  
}

# the the best run for a waypoint
bestRun <- function (cdf, waypoint) {
  w <- cdf[[waypoint]]
  if (length (w) > 0){
    bestIdx <- which.min (w[length (w[,1]),1:50])
    w[,bestIdx]
  }
}
convergence_best = vector ("list", length (waypoints))
for (way in waypoints) {
  convergence_best[[way]] <- bestRun (convergence, way)
}

# the worst run for a waypoint
worstRun <- function (cdf, waypoint) {
  w <- cdf[[waypoint]]
  if (length (w) > 0){
    bestIdx <- which.max (w[length (w[,1]),1:50])
    w[,bestIdx]
  }
}
convergence_worst = vector ("list", length (waypoints))
for (way in waypoints) {
  convergence_worst[[way]] <- worstRun (convergence, way)
}


# Get number of steps
extractSteps <- function (df, path, waypoint, run) {
  w <- getByWaypoint (df, path, waypoint)
  r <- w[w$V4 == run, ]
  as.numeric (r$V5)
}
steps <- extractSteps (df, 1, 1, 1)

# Plot
addlinetoplot <- function(dataset, varx, vary, color) { 
  list(
    geom_line(data=dataset, aes_string(x=varx, y=vary), colour = color)
  )
}

library(gridExtra)
p <- list ()
for (way in waypoints){
  titleA = paste0 ("waypoints = ", way)
  titleB = paste0 ("PSO dimensions = ", way * 2)
  title = paste (titleA, titleB, sep = "\n")

  d <- data.frame(fitness = unlist(convergence_avg[[way]]), 
                iteration = steps)
  p[[way-1]] <-ggplot(d,aes(x = iteration, y = fitness)) + 
    geom_line(colour="#009E73") + xlim(0, 50000) + ylim(100, 500) +
    labs(title=title) + theme(axis.text.x=element_text(angle = 90, vjust = 0.5))
  
  d2 <- data.frame (fitness = unlist (convergence_best[[way]]),
                    iteration = steps)
  
  p[[way-1]] <- p[[way-1]] + addlinetoplot(d2, varx = "iteration", vary = "fitness", color = "#CC79A7")
  
  d3 <- data.frame (fitness = unlist (convergence_worst[[way]]),
                   iteration = steps)
  
  p[[way-1]] <- p[[way-1]] + addlinetoplot(d3, varx = "iteration", vary = "fitness", color = "#D55E00")
  
}

do.call (grid.arrange, p)



#############
# Scratch pad
#############

# N = 3
p[[2]] + xlim (0, 1500) + ylim (100, 450)
s <- 1500 / 10
ca_3 <- convergence_avg[[2]][1:s]
cb_3 <- convergence_best[[2]][1:s]
cw_3 <- convergence_worst[[2]][1:s]
c_3 <- rbind.data.frame(ca_3, cb_3, cw_3)
colnames (c_3) <- seq (from = 1, to =  1500, by = 10)


# N = 4
s <- 15000 / 10
p[[3]] + xlim (0, 15000) + ylim (100, 450)
ca_4 <- convergence_avg[[3]][1:s]
cb_4 <- convergence_best[[3]][1:s]
cw_4 <- convergence_worst[[3]][1:s]
c_4 <- rbind.data.frame (ca_4, cb_4, cw_4)
colnames (c_4) <- seq (from = 1, to = 15000, by = 10)

# N = 5
s <- 50000 / 10
p[[4]] + xlim (0, 50000) + ylim (100, 450)
ca_5 <- convergence_avg[[4]][1:s]
cb_5 <- convergence_best[[4]][1:s]
cw_5 <- convergence_worst[[4]][1:s]
c_5 <- rbind.data.frame (ca_5, cb_5, cw_5)
colnames (c_5) <- seq (from = 1, to = 50000, by = 10)


################
## Infeasible ##
################
infeasibleRunIdxs <- list ()
for (way in waypoints) {
  infeasibleRunIdxs[[way]] <- setdiff (runs, feasibleRunIdxs[[way]])
  
}

dfNFeasible <- data.frame (V1=character(), V2=character(), V3=character(), V4=character(), V5=character(), V6=character())
for (w in waypoints){
  dfw <- df[df$V3 == w, ]
  dfwF <- dfw[dfw$V4 %in% infeasibleRunIdxs[[w]], ]
  if (length (dfwF) != 0){
    dfNFeasible <- rbind (dfNFeasible, dfwF)
  }
  else{
    NULL
  }
}

# Get a list where each elem is a table of runs and steps for each waypoint
convergence = vector("list", length (waypoints)) 
for (way in waypoints) {
  convergence[[way]] <- sapply (X = infeasibleRunIdxs[[way]], FUN = function (x) {extractRun (dfNFeasible, 1, way, x)})
}


convergence_avg = vector("list", length (waypoints))
for (way in waypoints[3:9]) {
  convergence_avg[[way]] <- avgRun (convergence, way)
  
}

convergence_best = vector ("list", length (waypoints))
for (way in waypoints[3:9]) {
  convergence_best[[way]] <- bestRun (convergence, way)
}


convergence_worst = vector ("list", length (waypoints))
for (way in waypoints[3:9]) {
  convergence_worst[[way]] <- worstRun (convergence, way)
}


p <- list ()
for (way in waypoints[3:9]){
  titleA = paste0 ("waypoints = ", way)
  titleB = paste0 ("PSO dimensions = ", way * 2)
  title = paste (titleA, titleB, sep = "\n")
  
  d <- data.frame(fitness = unlist(convergence_avg[[way]]), 
                  iteration = steps)
  p[[way-3]] <-ggplot(d,aes(x = iteration, y = fitness)) + 
    geom_line() + xlim(0, 50000) + ylim(100, 500) +
    labs(title=title) + theme(axis.text.x=element_text(angle = 90, vjust = 0.5))

  d2 <- data.frame (fitness = unlist (convergence_best[[way]]),
                    iteration = steps)
  
  p[[way-3]] <- p[[way-3]] + addlinetoplot(d2, varx = "iteration", vary = "fitness", color = "#CC79A7")
  
  d3 <- data.frame (fitness = unlist (convergence_worst[[way]]),
                    iteration = steps)
  
  p[[way-3]] <- p[[way-3]] + addlinetoplot(d3, varx = "iteration", vary = "fitness", color = "#D55E00")
}

do.call (grid.arrange, p)
















#### Open Room. Experiment 2:  (40, 65) --> (125, 85)