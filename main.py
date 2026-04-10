import util

input = [
    ['r', 'r', 'r'],
    ['b', 'b', 'b'],
    ['y', 'y', 'y']
]

# 3x3 grid centered on (x,y)=(3,0)
# i dont know if this is how coordinates are defined in coppellia or not... i can check later
output_locations = [
    # [x, y, z]
    [4, -1, 0], [4, 0, 0], [4, 1, 0], 
    [3, -1, 0], [3, 0, 0], [3, 1, 0], 
    [2, -1, 0], [2, 0, 0], [2, 1, 0]
]

# only include three blocks of each color and track their availability and starting locations
block_metadata = {
    'y': [ # left of sheet
        {
            'available': True,
            'start_position': [4, -3, 0],
            'current_position': [4, -3, 0]
        },
        {
            'available': True,
            'start_position': [3, -3, 0],
            'current_position': [3, -3, 0]
        },
        {
            'available': True,
            'start_position': [2, -3, 0],
            'current_position': [2, -3, 0]
        }
    ],
    'r': [ # above sheet
        {
            'available': True,
            'start_position': [6, -1, 0],
            'current_position': [6, -1, 0]
        },
        {
            'available': True,
            'start_position': [6, 0, 0],
            'current_position': [6, 0, 0]
        },
        {
            'available': True,
            'start_position': [6, 1, 0],
            'current_position': [6, 1, 0]
        }
    ],
    'b': [ #right of sheet
        {
            'available': True,
            'start_position': [4, 3, 0],
            'current_position': [4, 3, 0]
        },
        {
            'available': True,
            'start_position': [3, 3, 0],
            'current_position': [3, 3, 0]
        },
        {
            'available': True,
            'start_position': [2, 3, 0],
            'current_position': [2, 3, 0]
        }
    ]
}

 # ROBOT SHOULD BE CENTERED AT [0, 0, 0] FOR SIMPLICITY! - i think... offsets of each arm length might make us want the start of the end effector to be (x,y)=(0,0) instead
# all blocks should be in the same general area, but all on the base level z=0
# there should only be 3 blocks of each color, therefore the scene is easier to create
# and it is easier to program where the robot should 'look' for the blocks
# ---> the blocks are going to be in defined locations, so the robot should just know where it is,
#   where it needs to go, then do its operations and interpolate to (x,y) positions between the two spots

# How the robot works: 
# begin at (0,0,5) iterate through input colors that go to specific output locations
# choose a color block location to go grab... do calculations to get the angles and distances you need to be
# rotate base, extend the horizontal arm, extend the vertical arm
# grab block, detract the vertical arm
# rotate base to output angle, extend / detract the horizontal arm to ouput (x,y), lower block to ground level and let go
# repeat for all input values

def reset():
    for block_color in block_metadata.keys():
        for block in block_metadata[block_color]:
            block['current_position'] = block['start_position'][:]
            print(block_color, block['current_position'])



if __name__ == "__main__":

    reset()

    print("working")