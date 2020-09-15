---
layout: page
title: RelationExtractorAnnotator
keywords: relation
permalink: '/relation.html'
nav_order: 6
parent: Full List Of Annotators
---

## Description

Stanford relation extractor is a Java implementation to find relations between two entities. The current relation extraction model is trained on the relation types (except the 'kill' relation) and data from the paper Roth and Yih, Global inference for entity and relation identification via a linear programming formulation, 2007, except instead of using the gold NER tags, we used the NER tags predicted by Stanford NER classifier to improve generalization. The default model predicts relations <tt>Live\_In</tt>, <tt>Located\_In</tt>, <tt>OrgBased\_In</tt>, <tt>Work\_For</tt>, and <tt>None</tt>. 

| Property name | Annotator class name | Generated Annotation |
| --- | --- | --- |
| relation | RelationExtractorAnnotator | MachineReadingAnnotations.RelationMentionsAnnotation | 

## More information 

For more details of how to use and train your own model, see [this page](http://nlp.stanford.edu/software/relationExtractor.html)
