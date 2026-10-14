# Markdown guide conventions

These shared conventions guide agents editing Markdown documentation. Follow the
repository’s `AGENTS.md` for their scope and any repository-specific exceptions.

Use GitHub-compatible Markdown and GitHub heading-anchor conventions.

## Writing and formatting

- Use plain English, active voice, and consistent terminology.
- Write short paragraphs, each focused on one idea.
- Use numbered lists for procedures and bullets for unordered items.
- Use tables for comparisons or structured reference information.
- Use fenced code blocks with a language identifier.
- Use inline code for commands, filenames, configuration keys, and placeholders.
- Use bold sparingly to highlight important concepts.
- Leave a blank line around headings, lists, tables, and code blocks.

## Titles and heading hierarchy

- Use exactly one H1 (`#`) as the document title.
- Make titles specific and descriptive, such as "Configure backups."
- Use sentence case for titles and headings.
- Use H2 (`##`) for main sections and H3 (`###`) for subsections.
- Do not skip heading levels.
- Avoid deeper nesting unless necessary.
- Keep headings unique so internal links are unambiguous.

## Guide structure

Use the following order where relevant:

1. Title
2. Short summary
3. Table of contents for longer guides
4. Prerequisites
5. Main instructions or explanation
6. Verification or expected results
7. Troubleshooting
8. Related resources

Omit sections that do not help the reader.

## Summary and navigation

- Immediately below the title, write 2–3 sentences explaining the
  guide's purpose, intended audience, and expected outcome.
- Add a linked table of contents when the guide has five or more
  main sections.
- Link to H2 headings; include H3 headings only when useful.
- Follow the target Markdown renderer's heading-anchor conventions.
- Update the table of contents whenever headings change.
- Do not duplicate navigation provided automatically by the publishing system.

## Links

- Use descriptive link text, such as "Backup configuration,"
  rather than "click here."
- Use relative links for documents and assets in the repository.
- Link to a specific section when it helps the reader.
- Check that linked files and internal heading anchors exist.
- End with relevant related resources when useful.

## Visuals

- Add diagrams, screenshots, or charts when they improve understanding.
- Place visuals near the text they explain.
- Provide descriptive alt text.
- Keep essential instructions available in text.
