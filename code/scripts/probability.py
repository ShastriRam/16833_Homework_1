from scipy.stats import expon
import scipy.signal.gaussian as gauss
numSamples = 1000
stdev = 1	# Adjusts the width of the gaussian
exponentialScaleFactor = 1/50   # The sum of the exponential curve that I am generating is ~21.6
uniformValue = .05



# To make things fast, the exponential function should be precomputed as a lookup table
# It should then be scaled to get it to be the correct size relative to the other distributions.
# The gaussian should also be precomputed but with twice the width of the ranges.  
# The gaussian should have things in the form   [pr(x),  sum of this and all previous probabilities]
# The exponential function should be calculated the same way.
# I'm not going to use the delta function because I am not seeing it in the data.
# The uniform distribution should have [pr(x), pr(x) * 1000]


# To get a sample (A)
# * Lookup the exponential probability if the actual measurement is less than the particle's measurement
# * Lookup the gaussian probability @ sample location and add to previous value
# * Add in the uniform probability

# Now find the sum of the whole PDF (B)
# * Lookup the exponential sum at the particle's measurement
# * Find the sum of the gaussian by subtracting the sum from the sample just to the left of the PDF from the last used sample's sum
# * Add the previous two values from the uniform distributions sum.

# Divide A/B to get the probability
# Take the log of that value 

# Make the exponential distribution
x = np.linspace(expon.ppf(0.01), expon.ppf(0.99), numSamples) # Makes numSamples samples with a probability ranging from .99 to .1
expPDF = expon.pdf(x)
expPDF *= exponentialScaleFactor # Scale it down so that 

# Make the gaussian distribution
gaussPDF = gauss(numSamples * 2,stdev,True) 


# Resize the distributions to give them a second row.
expPDF.resize(2,numSamples)
gaussPDF.resize(2,numSamples*2)


# Find the sums at each point in the two PDFs
for I in range(numSamples):
	expPDF[1][I] = expPDF[0][0:I+1].sum()

for I in range(numSamples*2):
	gaussPDF[1][I] = gaussPDF[0][0:I+1].sum()

uniformSum = uniformValue * numSamples;



# Generate probabilities.  Each bin in the PDF for each distribution represents a 10cm bit of space

# To get a sample (A)
# * Add in the uniform probability
# * Lookup the exponential probability if the actual measurement is less than the particle's measurement
# * Lookup the gaussian probability @ actual measurement and add to previous value


# Now find the sum of the whole PDF (B)
# * Lookup the exponential sum at the particle's measurement
# * Find the sum of the gaussian by subtracting the sum from the sample just to the left of the PDF from the last used sample's sum
# * Add the previous two values to the uniform distribution's sum.

# Divide A/B to get the probability
# Take the log of that value 

actualMeasurement = 206 # cm
particleMeasurement = 193 

# Adjust the measurements into 10cm divisions ie: Convert them into their bin locations
actualMeasurement = round(actualMeasurement/10)
particleMeasurement = round(particleMeasurement/10)


probability = uniformValue;
if actualMeasurement <= particleMeasurement:
	probability += expPDF[0][actualMeasurement]

# Figure out the shift of the gaussian.
# As is, the center is at sample numSamples 
firstGaussianSample = numSamples - particleMeasurement
lastGaussianSample = firstGaussianSample + numSamples

probability += gaussPDF[0][firstGaussianSample + actualMeasurement]



# find the sum of the whole PDF to divide by
normalizer = uniformSum
normalizer += expPDF[1][particleMeasurement]
normalizer += gaussPDF[1][lastGaussianSample] - gaussPDF[1][firstGaussianSample - 1]

# Calculate the actual probability
probability /= normalizer

# Now find the log value of the probability
logProbability = math.log(probability)










