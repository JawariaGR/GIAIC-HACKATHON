import React, { useEffect } from 'react';
import type { RootProps } from '@docusaurus/theme-common';
import { SelectedTextProvider, useSelectedText } from '@site/src/contexts/SelectedTextContext';

// This is the place where you can inject components that will wrap your whole application.
// For example, you can setup react context providers here.
//
// In this file, you should import your Context providers and wrap the
// <WrappedRoot> component with them.

function RootWrapper({ children }: RootProps): JSX.Element {
  return (
    <SelectedTextProvider>
      <SelectedTextListener>
        {children}
      </SelectedTextListener>
    </SelectedTextProvider>
  );
}

// Separate component to use the context
function SelectedTextListener({ children }: { children: React.ReactNode }): JSX.Element {
  const { setSelectedText } = useSelectedText();

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const text = selection ? selection.toString().trim() : '';
      if (text) {
        setSelectedText(text);
      } else {
        setSelectedText(null); // Clear selection if nothing is selected
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [setSelectedText]);

  return <>{children}</>;
}

export default RootWrapper;