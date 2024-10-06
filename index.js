document.getElementById('momentForm').addEventListener('submit', function(e) {
    e.preventDefault();
    
    // Get user inputs
    let beams = document.getElementById('beams').value.split(',').map(Number);
    let columns = document.getElementById('columns').value.split(',').map(Number);
    let udl = document.getElementById('udl').value.split(',').map(Number);
    let numColumns = parseInt(document.getElementById('column_no').value);

    // Check inputs
    if(beams.length !== numColumns + 1) {
        alert("Please ensure correct input lengths.");
        return;
    }
    
    let distributionFactors = calculateDistributionFactors(beams, columns);
    let fixedEndMoment = calculateFixedEndMoments(beams, udl);
    let finalMoments = momentDistribution(distributionFactors, fixedEndMoment, beams);
    console.log(distributionFactors);
    displayResults(finalMoments);
});


let test;

// Calculate distribution factors for each joint
function calculateDistributionFactors(beams, columns) {
    let distributionFactors = [];
    let totalStiffness;

    for (let i = 0; i < beams.length-1; i++) { // For DF of middle joints

        totalStiffness = 1/beams[i] + 1/beams[i + 1] + 1/columns[0] + 1/columns[1];
        distributionFactors.push([1 / beams[i] / totalStiffness, 1 / beams[i + 1] / totalStiffness]);  
    }

    const DF_i = Math.pow(beams[0], -1) / (Math.pow(beams[0], -1) + Math.pow(columns[0], -1) + Math.pow(columns[1], -1));
    const DF_f = Math.pow(beams[beams.length - 1], -1) / (Math.pow(beams[beams.length - 1], -1) + Math.pow(columns[0], -1) + Math.pow(columns[1], -1));

    distributionFactors.splice(0,0,DF_i);   // Insert of initial and last 3-face joint in DF
    distributionFactors.splice(distributionFactors.length,0,DF_f);

    console.log(distributionFactors);
    
    return distributionFactors;
    
}



// Function to calculate fixed end moments due to UDL on beams
function calculateFixedEndMoments(beams, udl) {
    
    let fixedEndMoments = [];

    for (let i = 0; i < beams.length - 1; i++) { // For DF of middle joints
        let w = [udl[i], udl[i+1]];  // UDLs (in kN/m)
        let L = [beams[i], beams[i+1]];  // Length of the beams (in meters)

        // Fixed end moments formulas
    let FEM_BA = (w[0] * Math.pow(L[0], 2)) / 12;  // Moment at end A
    let FEM_BC = -(w[1] * Math.pow(L[1], 2)) / 12;   // Moment at end B

        fixedEndMoments.push([ FEM_BA, FEM_BC ]);  // FEMs on the both side of a joint
    }

    const FEM_i = -(udl[0] * Math.pow(beams[0], 2)) / 12;  // Moment at Initial end 
    const FEM_f = (udl[udl.length - 1] * Math.pow(beams[beams.length - 1], 2)) / 12;   // Moment at Final end 

    fixedEndMoments.splice(0,0,FEM_i);   // Insert of FEMs in initial and last 3-face joint
    fixedEndMoments.splice(fixedEndMoments.length,0,FEM_f);

    console.log(fixedEndMoments);

    return fixedEndMoments;
}



  // Function for 1st and last joint Carry-Over Moments Distribution
function COM_By_F_L(COMs,balance,i,carryOverFactor) {
    // Carry-over moments
   if (i===0) {
       COMs[1][0] += (balance * carryOverFactor);

   } else {
       COMs[i-1][1] += (balance * carryOverFactor);
   };
}



// Function for 2st and 2nd last joint Carry-Over Moments Distribution
function COM_For_F_L (COMs,left,right,i,carryOverFactor,distributionFactors) {
    if (i==1) {
        COMs[0] += left * carryOverFactor;
        COMs[2][0] += right * carryOverFactor;
    } else if (i==distributionFactors.length - 2) {
        COMs[i-1][1] += left * carryOverFactor;
        COMs[i+1] += right * carryOverFactor;
    } else {
        COMs[i-1][1] += left * carryOverFactor;
        COMs[i+1][0] += right * carryOverFactor;
    }
}



// Perform moment distribution until equilibrium
function momentDistribution(distributionFactors, moments, beams) {
    const carryOverFactor = 0.5;
    let TotalMoments = [...moments];
    let iterations = 6; // Limit the number of iterations
    let tolerance = 0.01; // Stop when moments are small enough
    let corrections = moments;// corrections will carry COM due to moment Distributions 

    for (let iteration = 0; iteration < iterations; iteration++) {
        // Distribution array is made empty zero
        let Distributions = moments.map(item => 
            Array.isArray(item) ? item.map(() => 0) : 0
          ); 
            console.log(Distributions);

        // Distribution array is made empty zero
        let COMs = moments.map(item => 
            Array.isArray(item) ? item.map(() => 0) : 0
          );    

          // Apply moment distributions and carry-over effects
        for (let i = 0; i < distributionFactors.length; i++) {
            if (i === 0 || i === (distributionFactors.length - 1) ) {
                let balance = (-corrections[i]) * distributionFactors[i];
                Distributions[i] += balance;
                COM_By_F_L(COMs,balance,i,0.5);
            } else {
                let balancingMoment = -(corrections[i][0] + corrections[i][1]) ;
                let left = balancingMoment * distributionFactors[i][0]; 
                let right = balancingMoment * distributionFactors[i][1];

                // Adjust the moments (Balancing step)
                Distributions[i][0] += left;
                Distributions[i][1] += right;

                COM_For_F_L(COMs,left,right,i,0.5,distributionFactors)
            }
        }
         // Corrections Will carry COMs of joints
         corrections = moments.map(item => 
            Array.isArray(item) ? item.map(() => 0) : 0
          );


        corrections = COMs;
        // Update unbalanced moments and check tolerance
        let maxMoment = Math.max(...corrections.map(Math.abs));
        if (maxMoment < tolerance) {
            break;
        }
        
        // Update unbalanced moments
        for (let i = 0; i < 2; i++) {
            let addMoments = (i===0)
                ? (corrections)
                : (Distributions);
            TotalMoments = TotalMoments.map((item, index) => {
                if (Array.isArray(item)) {
                // Add corresponding elements of inner arrays
                return item.map((innerItem, innerIndex) => innerItem + addMoments[index][innerIndex]);
                } else {
                // Add the numbers directly
                return item + addMoments[index];
                }
            });
        }

    }
    console.log(TotalMoments);
    return TotalMoments;
}

 // Display the results in the output section
 function displayResults(finalMoments) {
    console.log(finalMoments);
    let results = document.getElementById('results');
    results.innerHTML = "<h2>Final Moments at Joints:</h2><ul>";
    finalMoments.forEach((moment, index) => {
        if (Array.isArray(moment)) {
            // For joints with two moments (intermediate joints)
            results.innerHTML += `<li>Joint ${index + 1}: 
                Moment 1: ${moment[0].toFixed(2)} kNm, 
                Moment 2: ${moment[1].toFixed(2)} kNm</li>`;
        } else {
            // For joints with a single moment (initial and final joints)
            results.innerHTML += `<li>Joint ${index + 1}: ${moment.toFixed(2)} kNm</li>`;
        }
    });
    results.innerHTML += "</ul>";
}

// let array1 = [1,[2,3],[3,2],1];
// let array2 = [1,[2,3],[3,2],1];
// let add = array1+array2;
// console.log(add);

let array1 = [1,2,3,3,2,1];
let array2 = [1,2,3,3,2,1];
let add = array1+array2;
console.log(add);