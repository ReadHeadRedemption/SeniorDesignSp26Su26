// Navigation active link highlighting based on current page
document.addEventListener('DOMContentLoaded', function() {
    const navLinks = document.querySelectorAll('.nav-links a');
    const currentPage = window.location.pathname.split('/').pop() || 'index.html';
    
    // Set active link based on current page
    navLinks.forEach(link => {
        const linkPage = link.getAttribute('href');
        link.classList.remove('active');
        
        // Check if this link matches the current page
        if (linkPage === currentPage || 
            (currentPage === '' && linkPage === 'index.html') ||
            (currentPage === 'index.html' && linkPage === 'index.html')) {
            link.classList.add('active');
        }
    });

    // Smooth scrolling for any hash links on the same page
    const hashLinks = document.querySelectorAll('a[href^="#"]');
    hashLinks.forEach(link => {
        link.addEventListener('click', function(e) {
            e.preventDefault();
            const targetId = this.getAttribute('href').slice(1);
            const targetSection = document.getElementById(targetId);
            
            if (targetSection) {
                targetSection.scrollIntoView({
                    behavior: 'smooth',
                    block: 'start'
                });
            }
        });
    });
});

// Function to easily update project information
function updateProjectInfo(field, value) {
    const fieldMap = {
        'projectTitle': 'Update the project title in the Home section',
        'description': 'Update the project description',
        'sponsor': 'Update sponsor information',
    };
    
    console.log(`To update ${field}: ${value}`);
}

// Function to add a group member (for console use)
function addMember(name, role, bio, email, linkedin, photoUrl) {
    console.log('Add member to team.html:');
    console.log(`Name: ${name}, Role: ${role}`);
    console.log(`Bio: ${bio}`);
    console.log(`Email: ${email}, LinkedIn: ${linkedin}`);
    console.log(`Photo: ${photoUrl}`);
}

// Export functions for console access
window.projectSite = {
    addMember: addMember,
    updateProjectInfo: updateProjectInfo
};
