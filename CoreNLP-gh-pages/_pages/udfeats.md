---
layout: page
title: UDFeatureAnnotator
keywords: udfeats
permalink: '/udfeats.html'
nav_order: 8
parent: Full List Of Annotators
---

## Description

Labels tokens with their [Universal Dependencies](http://universaldependencies.org/) universal part of speech (UPOS) and features.

This is a highly specialist annotator. At the moment it only works for English. It requires a constituency parse to produce UPOS tags (but can add UD features based on dependency parses). Of our provided text output formats, the annotations this annotator produces only appear in the `conllu` format. This annotator may well disappear in a future release of CoreNLP once UD is further integrated into processing.

| Property name | Annotator class name | Generated Annotation |
| --- | --- | --- |
| udfeats | UDFeatureAnnotator | CoNLLUFeats, CoarseTagAnnotation |

## Options

At present, there are no options.

## More information 

See [the Universal Dependencies documentation](http://universaldependencies.org/) for more information.
