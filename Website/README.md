# C.R.I.P Senior Design Project Website

## Overview
This is a multi-page website for the Conductive Rapid Inking Prototyping (C.R.I.P) senior design project.

## File Structure
```
project/
├── index.html          (Home page)
├── team.html           (Team members with photos)
├── sd1.html            (SD1 deliverables)
├── sd2.html            (SD2 deliverables)
├── materials.html      (Additional materials)
├── styles.css          (Styling)
├── script.js           (JavaScript functionality)
├── images/             (Photo folder - CREATE THIS)
│   ├── ethan-baker.jpg
│   ├── andrew-eppich.jpg
│   ├── jeremy-jiang.jpg
│   ├── giancarlo-luna.jpg
│   ├── team-photo.jpg
│   ├── placeholder.jpg
│   └── placeholder-wide.jpg
└── Documents/          (Your documents folder)
    └── SD1/
        └── D&C Submission - Group 7.pdf
```

## Adding Team Photos

### 1. Create the Images Folder
Create a folder named `images` in the same directory as your HTML files.

### 2. Add Individual Photos
Add photos for each team member with these exact filenames:
- `ethan-baker.jpg` - Ethan Baker's photo
- `andrew-eppich.jpg` - Andrew Eppich's photo
- `jeremy-jiang.jpg` - Jeremy Jiang's photo
- `giancarlo-luna.jpg` - Giancarlo Luna's photo

**Photo Guidelines:**
- Recommended size: 800x800 pixels (square)
- Format: JPG or PNG
- Professional headshot or team photo style
- Good lighting and clear face visibility

### 3. Add Team Photo
Add a full team photo as `team-photo.jpg`:
- Recommended size: 1200x600 pixels (landscape)
- Format: JPG or PNG
- Shows all team members together

### 4. Create Placeholder Images (Optional)
If you don't have photos yet, create simple placeholder images:
- `placeholder.jpg` - 800x800 pixels with text "Photo Coming Soon"
- `placeholder-wide.jpg` - 1200x600 pixels with text "Team Photo Coming Soon"

The website will automatically fall back to placeholders if photos aren't found.

## Customizing Content

### Update Team Member Information (team.html)
Edit the following for each team member:
- **Bio**: Replace the placeholder text in `<p class="member-bio placeholder">`
- **Email**: Update the href in `<a href="mailto:...">`
- **LinkedIn**: Update the href in the LinkedIn link

### Update Project Description (index.html)
- Find the "Project Description" section
- Replace placeholder text with your actual project description
- Update "Key Features" and "Project Goals" sections

### Add Documents
1. Keep your documents in the `Documents/SD1/` and `Documents/SD2/` folders
2. Update the href attributes in sd1.html and sd2.html to point to your files
3. Change "Not yet uploaded" to descriptive text once files are added

## Navigation
The website has 5 pages:
- **Home** - Project overview and general information
- **Team** - Team member profiles with photos
- **SD1** - Senior Design 1 deliverables
- **SD2** - Senior Design 2 deliverables
- **Materials** - Additional resources and documentation

## Uploading to a Server
To make your website live:
1. Ensure all files are in the correct structure
2. Upload all files to your web hosting service
3. Make sure the images folder and all photos are uploaded
4. Test all links and photo displays

## Tips
- Photos display better when they're high quality but optimized for web (under 500KB each)
- Use consistent photo styles (all headshots or all candid, etc.)
- Test the website locally by opening index.html in a browser
- All placeholder text is marked with `class="placeholder"` for easy identification

## Support
For issues or questions, contact the team leads.
