// Navigation active link highlighting
document.addEventListener('DOMContentLoaded', function() {
    const navLinks = document.querySelectorAll('.nav-links a');
    
    // Highlight active nav link based on scroll position
    window.addEventListener('scroll', function() {
        const sections = document.querySelectorAll('section');
        let current = '';
        
        sections.forEach(section => {
            const sectionTop = section.offsetTop;
            const sectionHeight = section.clientHeight;
            if (scrollY >= sectionTop - 200) {
                current = section.getAttribute('id');
            }
        });
        
        navLinks.forEach(link => {
            link.classList.remove('active');
            if (link.getAttribute('href').slice(1) === current) {
                link.classList.add('active');
            }
        });
    });

    // Smooth scrolling for navigation links
    navLinks.forEach(link => {
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
// Usage: updateProjectInfo('projectTitle', 'Your Project Name')
function updateProjectInfo(field, value) {
    const fieldMap = {
        'projectTitle': 'Update the project title in the Home section',
        'description': 'Update the project description',
        'sponsor': 'Update sponsor information',
        // Add more fields as needed
    };
    
    console.log(`To update ${field}: ${value}`);
    // You can expand this function to programmatically update the page
}

// Function to add a group member
// Usage: addMember('John Doe')
function addMember(name) {
    const membersList = document.getElementById('members-list');
    const newMember = document.createElement('li');
    newMember.textContent = name;
    membersList.appendChild(newMember);
}

// Function to add a reviewer
// Usage: addReviewer('Dr. Smith')
function addReviewer(name) {
    const reviewersList = document.getElementById('reviewers-list');
    const newReviewer = document.createElement('li');
    newReviewer.textContent = name;
    reviewersList.appendChild(newReviewer);
}

// Export functions for console access
window.projectSite = {
    addMember: addMember,
    addReviewer: addReviewer,
    updateProjectInfo: updateProjectInfo
};
