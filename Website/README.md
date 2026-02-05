# Senior Design Project Website

A clean, professional website template for your UCF Senior Design project. This site is designed to be easily editable and expandable as you add more deliverables throughout the semester.

## Files

- **index.html** - Main HTML file with all page structure
- **styles.css** - CSS styling for a modern, responsive design
- **script.js** - JavaScript for interactivity and navigation
- **README.md** - This file with instructions

## Quick Start

1. Open `index.html` in any web browser to view the website
2. Edit the placeholder text in `index.html` to add your project information

## How to Edit the Website

### Editing Project Information

Open `index.html` and find the section marked with `[placeholder text]`. Replace these with your actual information:

#### Home Page Section (id="home")
- **Project Title**: Replace `[Your Project Title Here]`
- **Group Members**: Replace member placeholders in the `<ul id="members-list">` section
- **Reviewers**: Replace reviewer placeholders in the `<ul id="reviewers-list">` section
- **Project Description**: Replace the paragraph in the "Project Description" card
- **Sponsorship**: Replace sponsor name and contact information

### Adding/Updating Deliverables

#### For SD1 Deliverables:
The following items are already structured in the SD1 section (id="sd1"):
- Divide and Conquer Document
- Midterm Milestone Report
- SD1 Final Report
- Mini Demo Video

To add a link to an uploaded file, edit the card in the HTML:
```html
<div class="deliverable-card">
    <h3>Document Title</h3>
    <p>Brief description or date uploaded</p>
    <a href="path/to/your/file.pdf" class="upload-placeholder">Download</a>
</div>
```

#### For SD2 Deliverables:
All 8 SD2 deliverables are already set up in the SD2 section (id="sd2"):
- CDR Presentation Video
- CDR Presentation Slides
- Midterm Demonstration Video
- 8-Page Conference Paper
- SD2 Final Report
- Final Presentation Video
- Final Presentation Slides
- Final Demonstration Video

#### For Additional Materials:
Add items to the "Additional Materials" section (id="materials") using the same card structure.

### Adding New Members or Reviewers (Using Browser Console)

You can add members or reviewers directly through the browser console:

```javascript
// Add a new group member
projectSite.addMember("Jane Doe");

// Add a new reviewer
projectSite.addReviewer("Dr. Johnson");
```

## File Organization Recommendations

When uploading documents and videos, consider organizing them in folders:

```
Website/
├── index.html
├── styles.css
├── script.js
├── README.md
├── documents/
│   ├── sd1/
│   │   ├── divide-and-conquer.pdf
│   │   ├── midterm-report.pdf
│   │   └── final-report.pdf
│   └── sd2/
│       ├── conference-paper.pdf
│       ├── final-report.pdf
│       └── ...
├── videos/
│   ├── sd1/
│   │   └── demo-video.mp4
│   └── sd2/
│       ├── cdr-presentation.mp4
│       ├── midterm-demo.mp4
│       └── ...
└── presentations/
    ├── cdr-slides.pptx
    └── final-slides.pptx
```

Then reference files like:
```html
<a href="documents/sd1/divide-and-conquer.pdf" class="upload-placeholder">Download PDF</a>
```

## Hosting the Website

### Local Viewing
Simply open `index.html` in any web browser.

### Online Hosting Options
- **GitHub Pages** (Free) - Upload files to a GitHub repository
- **UCF Web Server** - Check with your university for student hosting options
- **Netlify** (Free) - Drag and drop to deploy
- **Vercel** (Free) - Simple hosting for static sites

## Features

✅ Responsive design (works on mobile, tablet, desktop)
✅ Organized sections for Home, SD1, SD2, and Additional Materials
✅ Professional styling with hover effects
✅ Smooth navigation between sections
✅ Easy to update and expand
✅ All 17 deliverables pre-structured
✅ No backend or database required

## Customization Tips

1. **Change Colors**: Edit the color codes in `styles.css`
   - Primary color: `#3498db` (blue)
   - Secondary color: `#2c3e50` (dark gray)
   - Success color: `#27ae60` (green)

2. **Add Embedded Videos**: Replace video links with embedded video players:
   ```html
   <iframe width="100%" height="400" src="https://www.youtube.com/embed/VIDEO_ID" 
           frameborder="0" allowfullscreen></iframe>
   ```

3. **Add Images**: Insert project images:
   ```html
   <img src="path/to/image.jpg" alt="Description" style="max-width: 100%; height: auto;">
   ```

4. **Change Fonts**: Update the font-family in `styles.css`

## Need Help?

- Check the placeholder text for clues on what to update
- Look at the HTML structure to understand where content goes
- Use browser developer tools (F12) to inspect elements
- All styling is in `styles.css` for easy customization

## Good Luck with Your Senior Design Project!
