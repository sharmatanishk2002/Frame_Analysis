document.getElementById('momentForm').addEventListener('submit', function(e) {
    e.preventDefault();

    // Retrieve data from localStorage
    const columnData = JSON.parse(localStorage.getItem('columnData')) || [];
    const beamData = JSON.parse(localStorage.getItem('beamData')) || [];
    const wallData = JSON.parse(localStorage.getItem('wallData')) || [];
    const loadData = JSON.parse(localStorage.getItem('loadData')) || [];

    // Get required inputs
    let beamNos = [];
    for (let i = 0; i < loadData.length; i++) {
        beamNos.push(loadData[i].beam);
        
    };

    let beams = beamData.beamLengths || []; // Beam lengths
    
    let columns = [
        columnData.abovecolumnLength || 0, 
        columnData.belowcolumnLength || 0
    ]; // Top and bottom column lengths

    // Safely map UDL values
    let udl = beamNos.map((_, index) => 
        loadData[index] && loadData[index].udl 
            ? loadData[index].udl 
            : 0 // Default value if UDL is not present
    );

    let pointloads = beamNos.map((_, index) => 
        loadData[index].pointLoadPresence
        ? loadData[index].pointLoadValue 
        : 0 // Default value if UDL is not present
    );// point load on beams

    let plbeamno = pointloads.map((PL, index) => 
        loadData[index].pointLoadPresence
        ? loadData[index].beam
        : 0 // Default value if UDL is not present
    );// point load on beams; // no. of beams on which PL is acting

    let pointloadpositions = pointloads.map((PL, index) => 
        loadData[index].pointLoadPresence
        ? loadData[index].pointLoadPosition 
        : 0 // Default value if UDL is not present
    );// point load on beams; // no. of beams on which PL is acting
    
    let numColumns = [columnData.numColumns-2];
    const round = [10];
    udl = udl.map((el, index) => {
        let value = el;
        for (let i = 0; i < wallData.length; i++) {
                        
            if(wallData[i].wallNumber === index+1){
              value += Number(wallData[i].wallUDL)
             } else value += 0 // Default value if UDL is not present
        }
        return value
    });      // Added wall udl 


    // Check inputs
    function checkInputs() {
        // for beam and column number matching
        if(beams.length != ++numColumns) {
            alert("Please ensure correct input lengths of beams or column numbers between beams.");
            return;
        };
        // Point load related checks
        if (pointloadpositions.length!=plbeamno.length || plbeamno.length!=pointloads.length) {
            alert("Please ensure that all point loads are present.");
            return;
        }
        // for top and bottom column
        if (columns[0] == 0 || columns[1] == 0) {
            alert("Please ensure that this model is not workable for roof frame.");
            return;
        }
        // for udl and beam lengths check
        if (udl.length!=beams.length) {
            alert("Please ensure that the total number of beam lengths must be equal to number of udls applied.");
            return;
        }

    }
     // Check Alerts ................
     checkInputs();


    let distributionFactors = calculateDistributionFactors(beams, columns);
    // let distributionFactors = [0,[0.278,0.167], [0.167,0.278], 0];
    let fixedEndMoment = calculateFixedEndMoments(beams, udl, pointloads,plbeamno, pointloadpositions);
    let finalMoments = momentDistribution(distributionFactors, fixedEndMoment, round);
    
   
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

    return distributionFactors;
    
}



    // Function to calculate fixed end moments due to UDL on beams
    function calculateFixedEndMoments(beams, udl, pointload,plbeamno, valueofa) {
    
    let fixedEndMoments = [];
    let FEM_i = -(udl[0] * Math.pow(beams[0], 2)) / 12;  // Moment at Initial end 
    let FEM_f = (udl[udl.length - 1] * Math.pow(beams[beams.length - 1], 2)) / 12;   // Moment at Final end 

    for (let i = 0; i < beams.length - 1; i++) { // For DF of middle joints

        let w = [udl[i], udl[i+1]];  // UDLs (in kN/m)
        let L = [beams[i], beams[i+1]];  // Length of the beams (in meters)
        
            
        // Fixed end moments formulas
        let FEM_BA = (w[0] * Math.pow(L[0], 2)) / 12;  // Moment at side A
        let FEM_BC = (w[1] * Math.pow(L[1], 2)) / 12;   // Moment at side c

        // Filter the loads that act on the current beam
        
        for (let j = 0; j < pointload.length; j++) {
            if (plbeamno[j] == (i+1)) {
                FEM_BA += pointload[j]*valueofa[j]*Math.pow((beams[i]-valueofa[j]), 2)/Math.pow(beams[i], 2);
            } else if (plbeamno[j] == (i+2)) {
                FEM_BC += pointload[j]*(beams[i+1]-valueofa[j])*Math.pow(valueofa[j], 2)/Math.pow(beams[i+1], 2);
            } else if (plbeamno[j] == 1) {
                FEM_i += -1*pointload[j]*(beams[i+1]-valueofa[j])*Math.pow(valueofa[j], 2)/Math.pow(beams[i+1], 2);
            } else if (plbeamno[j] == beams.length) {
                FEM_f += pointload[j]*valueofa[j]*Math.pow((beams[i]-valueofa[j]), 2)/Math.pow(beams[i], 2);
            }
        }

        fixedEndMoments.push([ FEM_BA, (-1)*FEM_BC ]);  // FEMs on the both side of a joint
    }

    

    fixedEndMoments.splice(0,0,FEM_i);   // Insert of FEMs in initial and last 3-face joint
    fixedEndMoments.splice(fixedEndMoments.length,0,FEM_f);

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
function momentDistribution(distributionFactors, moments, round) {
    const carryOverFactor = 0.5;
    let TotalMoments = [...moments];
    let iterations = round; // Limit the number of iterations
    let tolerance = 0.00001; // Stop when moments are small enough
    let corrections = moments;// corrections will carry COM due to moment Distributions 

    for (let iteration = 0; iteration < iterations; iteration++) {
        // Distribution array is made empty zero
        let Distributions = moments.map(item => 
            Array.isArray(item) ? item.map(() => 0) : 0
          ); 

        // Carry Over Moments array is made empty zero
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

        corrections = COMs.map((item, index) => 
            Array.isArray(item) ? item.map(innerItem => innerItem) :item
          );
      
        

        // break this iterations for loop if maximum moment reaches tolerance
        let flattened = corrections.flat(Infinity); // [2, 2, -3, 2.3, -1.2, -0.5]
        let absoluteValues = flattened.map(Math.abs); // [2, 2, 3, 2.3, 1.2, 0.5]
        let maxMoment = Math.max(...absoluteValues);
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
    return TotalMoments;
}

    // Display the results in the output section
    function displayResults(finalMoments) {

    let results = document.getElementById('results');
    results.innerHTML = "<h2>Final Moments at Joints:</h2><ul>";
    finalMoments.forEach((moment, index) => {
        if (Array.isArray(moment)) {
            // For joints with two moments (intermediate joints)
            results.innerHTML += `<li>Joint ${index + 1}: 
                Moment Left to joint: ${moment[0].toFixed(2)} kNm, 
                Moment Right to joint: ${moment[1].toFixed(2)} kNm</li>`;
        } else {
            // For joints with a single moment (initial and final joints)
            results.innerHTML += `<li>Joint ${index + 1}: ${moment.toFixed(2)} kNm</li>`;
        }
    });
    results.innerHTML += "</ul>";
};

document.getElementById("columninfo").addEventListener("click", function () {
    // Open columninfo.html in a new tab
    window.open("columninfo.html", "_blank");
});