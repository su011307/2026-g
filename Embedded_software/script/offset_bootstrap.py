import numpy as np

EXPECTED_MEAN = 57.25
BOOTSTRAP_ITERATION = 1_0000
PATH_TO_DATA = '/home/shaoyafan/develop/wavegen/res/data/test_3_3.txt'

with open(PATH_TO_DATA) as f:
    line = f.readline()
    data = np.array([float(d) for d in line.split(',')])

offset = data - EXPECTED_MEAN
mean = np.empty(BOOTSTRAP_ITERATION)

for i in range(BOOTSTRAP_ITERATION):
    smpl = np.random.choice(
        offset,
        size=len(offset),
        replace=True
    )
    mean[i] = np.mean(smpl)

offset_estimate = np.mean(mean)
ci_5, ci_95 = np.percentile(
    mean,
    [5.0, 95.0]
)

print(
    f"estimated offset: {offset_estimate: .2f}\n",
    f"95% CI: [{ci_5: .2f}, {ci_95: .2f}]\n"
)
