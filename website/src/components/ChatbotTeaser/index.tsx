import React from 'react';
import Heading from '@theme/Heading';

function ChatbotTeaser(): React.JSX.Element {
  return (
    <section className="py-12 bg-gray-50 dark:bg-gray-800">
      <div className="container mx-auto px-4">
        <div className="max-w-4xl mx-auto">
          <div className="bg-white dark:bg-gray-700 rounded-xl shadow-md p-6 md:p-8 text-center">
            <div className="inline-flex items-center justify-center w-16 h-16 bg-blue-500 rounded-xl mb-4">
              <svg className="w-8 h-8 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 10h.01M12 10h.01M16 10h.01M9 16H5a2 2 0 01-2-2V6a2 2 0 012-2h14a2 2 0 012 2v8a2 2 0 01-2 2h-5l-5 5v-5z" />
              </svg>
            </div>

            <Heading as="h2" className="text-xl md:text-2xl font-bold text-gray-900 dark:text-white mb-3">
              Intelligent Learning Assistant
            </Heading>

            <p className="text-gray-600 dark:text-gray-300 mb-4">
              Coming soon: An AI-powered chatbot that answers questions based on the textbook content.
            </p>

            <div className="inline-block px-4 py-2 bg-blue-100 dark:bg-blue-900 text-blue-800 dark:text-blue-200 rounded-lg text-xs font-medium">
              Coming Soon
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default ChatbotTeaser;